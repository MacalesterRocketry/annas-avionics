//! SD card binary logging. Skeleton.
//!
//! Multicore design: the hot-loop tasks (sensor read, AHRS, control) call
//! `log_*` functions that just push a fixed-size `LogPacket` enum into an
//! Embassy SPSC channel. A dedicated `sd_writer_task` running on **core 1**
//! pops packets, serializes them via the writers in `crate::log_packets`,
//! and writes to the file. `dataFile.sync()` becomes an `f.flush().await`
//! every `SYNC_INTERVAL_MS` — non-blocking from core 0's perspective.
//!
//! This is the single biggest architectural win over the C++ version: in
//! C++, `dataFile.sync()` blocks the AHRS loop for ~30 ms whenever it fires.
//! On Embassy that 30 ms lives on core 1 and never touches the control path.

#![allow(dead_code, unused_variables)]

use core::fmt::Write;
use crate::config::board::SdConfig;
use crate::utils::errors::{Subsystem, mark_init_complete};
use chrono::DateTime;
use core::ops::ControlFlow;
use crc::Crc;
use defmt::Format;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi;
use embassy_rp::spi::{Blocking, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::Mode::ReadWriteCreate;
use embedded_sdmmc::{Directory, File, SdCard, SdCardError, TimeSource, Timestamp, VolumeManager};
use heapless::String;
use postcard::ser_flavors::crc::CrcModifier;
use postcard::ser_flavors::{Cobs, Slice};
use serde::{Deserialize, Serialize};
use crate::utils::errors::{SubsystemError, report_init_error, report_runtime_error};
use crate::utils::unwrap_infallible;

/// Everything that can go wrong in the SD card logging subsystem.
///
/// Neither `embedded_sdmmc::Error` nor `postcard::Error` is `Copy`, so neither
/// is this.
#[derive(Debug, Clone, Format)]
pub enum SdError {
    /// The card or its FAT filesystem reported an error.
    Fs(embedded_sdmmc::Error<SdCardError>),
    /// A `LogPacket` could not be serialized into the scratch buffer.
    Serialize(postcard::Error),
    /// Every log filename permitted by FAT 8.3 (`LOG0.BIN`..`LOG99999.BIN`) is
    /// taken, so there is nowhere left to write.
    LogDirFull,
    /// The SD card channel is full and cannot accept new log packets.
    LogChannelFull,
}

impl From<embedded_sdmmc::Error<SdCardError>> for SdError {
    fn from(err: embedded_sdmmc::Error<SdCardError>) -> Self {
        Self::Fs(err)
    }
}

impl From<postcard::Error> for SdError {
    fn from(err: postcard::Error) -> Self {
        Self::Serialize(err)
    }
}

impl SubsystemError for SdError {
    fn subsystem(&self) -> Subsystem {
        Subsystem::SD_CARD
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Format)]
pub enum LogEntry {
    GPS(crate::navigation::gps::GpsState),
    State(crate::state::FlightState),
    // Sensors(crate::sensors::SensorReadings),
    AHRS(crate::navigation::ahrs::AhrsState),
    Control(crate::state::ControlSetpoint),
    // TODO: Add some way to note PID, fin, and servo outputs (desired angular acceleration, desired deflection angle, and actual deflection angle).
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Format)]
pub struct LogPacket {
    pub timestamp_us: u64,
    pub entry: LogEntry,
}

pub fn log_packet(entry: LogEntry) {
    let packet = LogPacket {
        timestamp_us: Instant::now().as_micros(),
        entry,
    };
    match SD_LOGGING_CHANNEL.sender().try_send(packet) {
        Ok(_) => {}
        Err(_) => {
            SD_LOGGING_CHANNEL.sender().clear();
            // TODO: Maybe just clear the sensor values, not the system logs, and then clear the system logs if it's still full. System logs are more important.
            report_runtime_error(SdError::LogChannelFull); // TODO: Make sure this logs somehow to the SD.
        }
    }
    // TODO: Handle full channel. Maybe delete some packets? But should definitely mark that that's happened.
}

struct SDTimeSource {
    now: Instant,
    datetime: Option<DateTime<chrono::Utc>>,
}

impl TimeSource for SDTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        // TODO: this is a placeholder
        Timestamp::from_calendar(2026, 8, 29, 11, 08, 16).expect("Invalid date/time")
    }
}

pub static SD_LOGGING_CHANNEL: Channel<CriticalSectionRawMutex, LogPacket, 256> = Channel::new();

const CRC: Crc<u32> = Crc::<u32>::new(&crc::CRC_32_ISCSI);
pub async fn sd_logging_loop(sd_config: SdConfig) {
    let sd_receiver = SD_LOGGING_CHANNEL.receiver();

    let sd_card = init_sd_card(sd_config);
    let vol_mgr: RocketVolumeManager = VolumeManager::new(sd_card, SDTimeSource { now: Instant::now(), datetime: None });
    let file = match open_log_file(&vol_mgr) {
        Ok(file) => file,
        Err(e) => {
            // TODO: Do I need to unmount it or something here?
            report_init_error(e);
            return;
        }
    };

    const WRITE_BUFFER_SIZE: usize = 512;
    let mut write_buffer = [0u8; WRITE_BUFFER_SIZE];
    let mut buffer_index = 0;

    mark_init_complete(Subsystem::SD_CARD);
    loop {
        let packet = sd_receiver.receive().await;

        // TODO: figure out what the length of the serialized maximum packet is. Can that be programmatic?
        let buf = &mut [0u8; 128];
        // `Cobs::try_new` and `serialize_with_flavor` both fail with `postcard::Error`,
        // so `and_then` folds them into one result rather than panicking on the former.
        let ser_result = Cobs::try_new(Slice::new(buf)).and_then(|cobs| {
            postcard::serialize_with_flavor::<LogPacket, _, _>(
                &packet,
                CrcModifier::new(cobs, CRC.digest()),
            )
        });

        // TODO: add ability to close SD card
        match ser_result {
            Ok(serialized_packet) => {
                let mut packet_remaining_bytes = serialized_packet;

                while !packet_remaining_bytes.is_empty() {
                    let buffer_space_left = WRITE_BUFFER_SIZE - buffer_index;
                    let chunk_size = packet_remaining_bytes.len().min(buffer_space_left); // how much can be written before either the buffer is full or the packet is finished

                    write_buffer[buffer_index..buffer_index + chunk_size]
                        .copy_from_slice(&packet_remaining_bytes[..chunk_size]);

                    buffer_index += chunk_size;
                    packet_remaining_bytes = &mut packet_remaining_bytes[chunk_size..]; // chop off the part that's been written to the buffer

                    // If buffer is full, flush it and reset so we can log the rest of the packet
                    if buffer_index >= WRITE_BUFFER_SIZE { // should never be >, but might as well be safe
                        // TODO: This doesn't write by blocks, since it's in a file. Figure out what's most efficient.
                        if let Err(e) = file.write(&write_buffer) {
                            report_runtime_error(SdError::from(e));
                        }
                        // TODO: Should we write multiple blocks at once? Would require a small refactor.
                        buffer_index = 0;
                    }
                }
            }
            Err(e) => {
                report_runtime_error(SdError::from(e));
                // TODO: Should probably add some packet saying there was an error (though who knows if it can be logged)
            }
        }
    }
}

type RocketSdCard<'a> = SdCard<ExclusiveDevice<Spi<'a, SPI0, Blocking>, Output<'a>, Delay>, Delay>;
type RocketVolumeManager<'a> = VolumeManager<RocketSdCard<'a>, SDTimeSource>;
type RocketFile<'a, 'd> = File<'a, RocketSdCard<'d>, SDTimeSource, 4, 4, 1>;
fn open_log_file<'a, 'd>(vol_mgr: &'a RocketVolumeManager<'d>) -> Result<RocketFile<'a, 'd>, SdError> {
    let vol = vol_mgr.open_volume(embedded_sdmmc::VolumeIdx(0))?;
    let root_dir = vol.open_root_dir()?;

    let filename = choose_filename(&root_dir)?;

    // TODO: better handle if the file already exists somehow (maybe switch to using UUID or something)
    let file = root_dir.open_file_in_dir(
        filename.as_str(),
        ReadWriteCreate,
    )?;
    Ok(file)
}

fn init_sd_card<'d>(sd_config: SdConfig) -> RocketSdCard<'d> {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 50_000_000; // 50MHz
    // TODO: Needs to init a lot slower then move up. Look at SdCard docs.
    let spi = Spi::new_blocking(sd_config.spi, sd_config.sclk, sd_config.mosi, sd_config.miso, spi_config);
    let sd_cs = Output::new(sd_config.cs, Level::High);
    let spi_device = unwrap_infallible(ExclusiveDevice::new(spi, sd_cs, Delay));
    SdCard::new(spi_device, Delay)
}

fn choose_filename(root_dir: &Directory<RocketSdCard<'_>, SDTimeSource, 4, 4, 1>) -> Result<String<12>, SdError> {
    let mut highest_file_id: i32 = -1;

    root_dir.iterate_dir(|entry| {
        // 12 chars long; FAT stores 11 chars excluding the . for the extension
        let mut name_buf: String<12> = String::new();

        // Write the filename we're checking into the String buffer
        if write!(&mut name_buf, "{}", entry.name).is_ok() {

            // Capitalized because FAT is case-insensitive and stores everything as uppercase
            if name_buf.starts_with("LOG") && name_buf.ends_with(".BIN") {
                // Slice out the numeric portion (everything after "LOG" and before ".BIN")
                let number_slice = &name_buf[3..(name_buf.len() - 4)];

                if let Ok(val) = number_slice.parse::<i32>() {
                    if val > highest_file_id {
                        highest_file_id = val;
                    }
                }
            }
        }
        ControlFlow::Continue(())
    })?;

    let next_file_id = (highest_file_id + 1) as u32;

    let mut filename: String<12> = String::new();
    write!(&mut filename, "LOG{}.BIN", next_file_id).map_err(|_| SdError::LogDirFull)?;
    Ok(filename)
}
