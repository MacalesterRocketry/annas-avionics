//! See README.md for most of the details. This file initializes both cores and runs a set of
//! async loops on each core, using the Embassy framework.

#![no_std]
#![no_main]

extern crate uom;

use core::cell::RefCell;
use core::future::pending;
use crate::communication::sdcard::sd_logging_loop;
use crate::config::board::{GpsConfig, I2cConfig, IndicatorsConfig, InterruptConfig, PeripheralConfig, SdConfig};
use crate::config::board::ServoConfig;
use crate::control::control_loop;
use crate::navigation::gps::gps_loop;
use crate::state::{state_machine_loop, FlightState, SystemState};
use communication::indication::indicator_loop;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Executor, SpawnError, SpawnToken, Spawner};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, UART0};
use embassy_rp::{bind_interrupts, dma, pio};
use embassy_rp::{i2c, uart};
use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::watch::Watch;
use static_cell::StaticCell;
use crate::navigation::ahrs::ahrs_loop;
use crate::sensors::sensor_loops;

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
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>; // PIO for NeoPixels
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>; // DMA for NeoPixels
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>; // UART for GPS
}); // TODO: Extract these types to config.rs
// TODO: Add interrupt handlers for the sensors

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

pub static FLIGHT_STATE: Watch<CriticalSectionRawMutex, FlightState, 5> = Watch::new();

// TODO: Move this I2C0 out into a type variable
pub static I2C_BUS_CELL: StaticCell<Mutex<NoopRawMutex, RefCell<i2c::I2c<I2C0, i2c::Blocking>>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());
    let hw = take_hardware!(p);

    // spawn core 0 (the main core with everything fast)
    spawn_core(
        EXECUTOR0.init(Executor::new()),
        core0_main(hw.i2c, hw.interrupts, hw.servos),
    );

    // spawn core 1 (the secondary core with the slow stuff)
    spawn_core1(
        p.CORE1,
        // Needs to be unsafe because we're manually defining a chunk of memory to be the stack
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            spawn_core(
                EXECUTOR1.init(Executor::new()),
                core1_main(hw.sd, hw.gps, hw.indicators, hw.peripherals),
            )
        },
    );
}

fn spawn_core(
    executor: &'static mut Executor,
    core: Result<SpawnToken<impl Sized>, SpawnError>,
) -> ! {
    executor.run(|spawner| {
        let task = core.unwrap_or_else(|e| {
            error!("core: failed to spawn task: {:?}", e);
            defmt::panic!("core task spawn failed"); // If we can't start one of the cores, there's no recovering
        });

        spawner.spawn(task);
    });
}

/// Core 0: everything fast and time-sensitive
#[embassy_executor::task]
async fn core0_main(
    i2c_config: I2cConfig,
    interrupt_config: InterruptConfig,
    servos_config: ServoConfig,
) {
    info!("core 0: avionics task starting");

    embassy_futures::join::join4(
        state_machine_loop(),
        control_loop(servos_config),
        sensor_loops(i2c_config, interrupt_config),
        ahrs_loop(),
    ).await;
}

/// Core 1: everything slow or where timing is unimportant
#[embassy_executor::task]
async fn core1_main(
    sd_config: SdConfig,
    gps_config: GpsConfig,
    indicators_config: IndicatorsConfig,
    peripheral_config: PeripheralConfig,
) {
    info!("core 1: I/O task starting");

    embassy_futures::join::join4(
        indicator_loop(indicators_config),
        sd_logging_loop(sd_config),
        gps_loop(gps_config),
        supervisor_loop(peripheral_config),
    ).await;
}

// TODO: Move this function to a separate file after deciding if we actually want it
async fn supervisor_loop(peripheral_config: PeripheralConfig) -> ! {
    info!("initializing GPIO");
    let mut eject_button = Input::new(peripheral_config.eject_button, embassy_rp::gpio::Pull::Up);
    info!("GPIO initialized");

    // TODO: make a loop here
    //  Should handle eject button, battery monitoring, SD card full, etc.
    pending().await
}