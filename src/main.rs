//! Entry point + multicore executor setup.
//!
//! Core split (mirrors the porting plan):
//!   • Core 0 — sensor sampling, AHRS, state machine, roll PID, servo PWM.
//!     This is the hot loop; every µs matters.
//!   • Core 1 — SD card writes, GPS UART parsing, NeoPixel + buzzer.
//!     These can tolerate latency spikes; isolating them on a separate core
//!     keeps a 30 ms `sd.flush()` from stalling the control loop.

#![no_std]
#![no_main]

extern crate uom;

use core::cell::RefCell;
use crate::communication::sdcard::sd_logging_loop;
use crate::config::board::{GpsConfig, I2cConfig, IndicatorsConfig, InterruptConfig, PeripheralConfig, SdConfig};
use crate::config::board::ServoConfig;
use crate::control::control_loop;
use crate::navigation::gps::gps_loop;
use crate::state::{FlightState, system_loop};
use communication::indication::indicator_loop;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Executor, SpawnError, SpawnToken, Spawner};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, UART0};
use embassy_rp::{bind_interrupts, dma, pio};
use embassy_rp::{i2c, uart};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::watch::Watch;
use static_cell::StaticCell;

mod config;
mod navigation;
mod communication;
mod sensors;
pub mod control;
pub mod utils;
pub mod state;

// 8 KiB of stack for core 1. Lives in SCRATCH_X (see memory.x) so it doesn't
// share cache lines with the AHRS data on core 0. If a panic-probe trace shows
// core 1 stack overflow, bump this — there's 4 KiB of headroom in SCRATCH_X
// minus the linker-reserved bits.
#[unsafe(link_section = ".core1_stack")]
static mut CORE1_STACK: Stack<4096> = Stack::new();

// Bind the interrupt handler with the peripheral
bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>; // I2C for sensors
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>; // PIO for NeoPixels
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>; // DMA for NeoPixels
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>; // UART for GPS
}); // TODO: Extract these types to config.rs

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

pub static FLIGHT_STATE: Watch<CriticalSectionRawMutex, FlightState, 2> = Watch::new();

// TODO: Move this I2C0 out into a type variable
pub static I2C_BUS_CELL: StaticCell<Mutex<NoopRawMutex, RefCell<i2c::I2c<I2C0, i2c::Async>>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    FLIGHT_STATE
        .sender()
        .send(FlightState::PreLaunch(state::GroundSubState::Startup));

    let p = embassy_rp::init(Default::default());
    let hw = take_hardware!(p);

    // Spawn core 1's executor first so it's ready to receive packets the
    // moment core 0 starts producing them.
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            spawn_core(
                EXECUTOR1.init(Executor::new()),
                core1_main(hw.sd, hw.gps, hw.indicators),
            )
        },
    );

    spawn_core(
        EXECUTOR0.init(Executor::new()),
        core0_main(hw.i2c, hw.interrupts, hw.peripherals, hw.servos),
    );
}

fn spawn_core(
    executor: &'static mut Executor,
    core: Result<SpawnToken<impl Sized>, SpawnError>,
) -> ! {
    executor.run(|spawner| {
        let task = core.unwrap_or_else(|e| {
            error!("core: failed to spawn task: {:?}", e);
            defmt::panic!("core task spawn failed");
        });

        spawner.spawn(task);
    });
}

/// Core 0: everything fast and time-sensitive: sensors, AHRS, PID, servos, and the state machine.
#[embassy_executor::task]
async fn core0_main(
    i2c_config: I2cConfig,
    interrupt_config: InterruptConfig,
    peripheral_config: PeripheralConfig,
    servos_config: ServoConfig,
) {
    info!("core 0: avionics task starting");

    embassy_futures::join::join(
        system_loop(i2c_config, peripheral_config),
        control_loop(servos_config),
    ).await;
}

/// Core 1: everything slow or where timing is unimportant: GPS, SD writes, and indication.
#[embassy_executor::task]
async fn core1_main(
    sd_config: SdConfig,
    gps_config: GpsConfig,
    indicators_config: IndicatorsConfig,
) {
    info!("core 1: I/O task starting");

    // TODO:
    //   • init SD card (SPI1 @ 50 MHz, embedded-sdmmc::VolumeManager)

    embassy_futures::join::join3(
        indicator_loop(indicators_config),
        sd_logging_loop(sd_config),
        gps_loop(gps_config),
        // TODO: Add supervisor task for eject button, battery monitoring, SD card full, etc.?
    ).await;
}
