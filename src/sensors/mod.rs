//! Sensor I/O stuff. Each chip has a file wrapping its driver, which provides a struct
//! with `init()` and `read()` methods that uses an I2c bus type. These structs are then
//! owned by the `Sensors` struct, and failed reads go through a debouncer that decides
//! when a chip is considered faulted.

#![allow(dead_code, unused_variables)]

pub mod adxl375;
pub mod bmp390;
pub mod health;
pub mod lsm6dsox;

use crate::config::G;
use crate::config::board::InterruptConfig;
use crate::sensors::adxl375::AdxlReading;
use crate::sensors::bmp390::BmpReading;
use crate::sensors::health::{FaultDebouncer, SensorFault};
use crate::sensors::lsm6dsox::LsmReading;
use crate::utils::errors::{Subsystem, SubsystemError, mark_init_complete, report_init_error};
use crate::utils::math::{AngularVec3, Vec3};
use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_rp::gpio::{Input, Pull};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_hal::i2c::I2c;
use serde::{Deserialize, Serialize};

/// Owns every sensor driver instance sharing the I²C bus, plus the debouncer
/// deciding when that chip's read failures amount to a fault.
///
/// A driver is `None` when it failed to initialize.
pub struct Sensors<I2C: I2c> {
    adxl: Option<adxl375::Adxl<I2C>>,
    adxl_health: FaultDebouncer, // TODO: Maybe integrate these health values into the actual sensor reading?
    lsm: Option<lsm6dsox::Lsm<I2C>>,
    lsm_health: FaultDebouncer,
    bmp: Option<bmp390::Bmp<I2C>>,
    bmp_health: FaultDebouncer,
}

/// A failure attributable to one chip, which carries the driver's own error so defmt can actually
/// log what went wrong.
#[derive(Debug, defmt::Format)]
pub enum SensorError {
    Adxl(adxl375::Error),
    Lsm(lsm6dsox::Error),
    Bmp(bmp390::Error),
}

impl SubsystemError for SensorError {
    fn subsystem(&self) -> Subsystem {
        Subsystem::SENSORS
    }
}

/// Attempt to initialize every sensor on the shared I²C bus. Errors are marked individually,
/// but the overall sensor system can still be used even if some chips aren't working.
pub fn init_all<M: RawMutex, BUS: I2c>(interrupt_config: InterruptConfig, mutex: &'static Mutex<M, RefCell<BUS>>) -> Sensors<I2cDevice<'static, M, BUS>> {
    macro_rules! get_bus {
        ($mutex:expr) => {
            embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice::new($mutex)
        };
    }

    let adxl_int1 = Input::new(interrupt_config.adxl_int1, Pull::Down);
    let adxl_int2 = Input::new(interrupt_config.adxl_int2, Pull::Down);
    let adxl_bus = get_bus!(&mutex);
    let adxl = match adxl375::Adxl::init(adxl_bus, adxl_int1, adxl_int2) {
        Ok(driver) => Some(driver),
        Err(e) => {
            report_init_error(SensorError::Adxl(e));
            health::mark_faulted(SensorFault::ADXL);
            None
        }
    };
    let lsm_int1 = Input::new(interrupt_config.lsm_int1, Pull::Down);
    let lsm_int2 = Input::new(interrupt_config.lsm_int2, Pull::Down);
    let lsm_bus = get_bus!(&mutex);
    let lsm = match lsm6dsox::Lsm::init(lsm_bus, lsm_int1, lsm_int2) {
        Ok(driver) => Some(driver),
        Err(e) => {
            report_init_error(SensorError::Lsm(e));
            health::mark_faulted(SensorFault::LSM);
            None
        }
    };

    let bmp_int = Input::new(interrupt_config.bmp_int, Pull::Down);
    let bmp_bus = get_bus!(&mutex);
    let bmp = match bmp390::Bmp::init(bmp_bus, bmp_int) {
        Ok(driver) => Some(driver),
        Err(e) => {
            report_init_error(SensorError::Bmp(e));
            health::mark_faulted(SensorFault::BMP);
            None
        }
    };

    if health::faults().is_empty() {
        mark_init_complete(Subsystem::SENSORS);
    }

    Sensors {
        adxl,
        adxl_health: FaultDebouncer::new(SensorFault::ADXL),
        lsm,
        lsm_health: FaultDebouncer::new(SensorFault::LSM),
        bmp,
        bmp_health: FaultDebouncer::new(SensorFault::BMP),
    }
}

impl<I2C: I2c> Sensors<I2C> {
    /// Read every sensor that is answering. Returns biased + axis-corrected
    /// readings, with `None` for any chip that had nothing to give.
    /// TODO: Figure out how to handle different sensor speeds. Probably interrupts + queue, but unsure.
    pub async fn read_all(&mut self) -> SensorReadings {
        let adxl = match self.adxl.as_mut() {
            Some(adxl) => self
                .adxl_health
                .record(adxl.read().await.map_err(SensorError::Adxl)),
            None => None,
        };
        let lsm = match self.lsm.as_mut() {
            Some(lsm) => self
                .lsm_health
                .record(lsm.read().await.map_err(SensorError::Lsm)),
            None => None,
        };
        let bmp = match self.bmp.as_mut() {
            Some(bmp) => self
                .bmp_health
                .record(bmp.read().await.map_err(SensorError::Bmp)),
            None => None,
        };
        SensorReadings {
            adxl,
            lsm,
            bmp,
        }
    }
}

pub trait Sensor {
    type Bus: I2c;
    type Reading;
    type Error;

    /// Read the sensor, returning a bias-corrected reading in the body frame, generally in SI units.
    /// Returns `None` if the chip is not responding or has no new data.
    async fn read(&mut self) -> Result<Self::Reading, Self::Error> where Self: Sized;
}

pub trait AccelerometerReading {
    fn saturation_threshold_g(&self) -> f64;
    /// Return the body-frame linear acceleration (m/s²), bias-subtracted.
    fn accel(&self) -> Vec3;
    fn has_accel_saturated(&self) -> bool {
        self.accel().mag() >= (self.saturation_threshold_g() * 0.99 * G)
    }
}
pub trait GyroscopeReading {
    /// Return the body-frame angular rate (rad/s), bias-subtracted.
    fn gyro(&self) -> AngularVec3;
}
pub trait MagnetometerReading {
    /// Return the body-frame magnetic field (µT), bias-subtracted.
    fn mag(&self) -> Vec3;
}
pub trait TemperatureReading {
    /// Return the die temperature or outside temperature (°C).
    fn temperature(&self) -> f32;
}
pub trait PressureReading {
    /// Return the barometric pressure (Pa).
    fn pressure(&self) -> f32;
}
pub trait AltitudeReading {
    /// Return the barometric altitude (m).
    fn altitude(&self) -> f32;
}

/// One tick's worth of sensor data. A `None` field is a chip that didn't return any data
/// this tick, such as from an init or runtime failure or even just the sensor being
/// slower than the tick rate.
#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct SensorReadings {
    pub lsm: Option<LsmReading>,
    pub adxl: Option<AdxlReading>,
    pub bmp: Option<BmpReading>,
}

impl SensorReadings {
    /// Best-available acceleration for AHRS input: the low-G accel, unless it is saturated or
    /// absent, in which case the high-G stands in. `None` only when both accelerometers are gone.
    pub fn merged_accel(&self) -> Option<Vec3> {
        let high_g = self.adxl.map(|adxl| adxl.accel);
        match self.lsm {
            // Saturated, so the low-G reading is clipped: prefer the high-G if
            // there is one, otherwise keep the clipped value, which is at least
            // the right direction and a lower bound on the magnitude.
            Some(lsm) if lsm.has_accel_saturated() => high_g.or(Some(lsm.accel)),
            Some(lsm) => Some(lsm.accel),
            None => high_g,
        }
    }

    /// True if an accelerometer has recorded a launch-magnitude acceleration spike.
    pub fn has_launched(&mut self) -> bool {
        // TODO: Make this more robust, like latching and requiring a minimum duration.
        self.check_launch_accel()
    }

    fn check_launch_accel(&self) -> bool {
        self.merged_accel().is_some_and(|accel| {
            accel.mag() >= crate::config::LAUNCH_ACCEL_THRESHOLD_G * G
        })
    }
}

pub(crate) fn transform_sensor_axes(raw: Vec3) -> Vec3 {
    Vec3 {
        x: -raw.x,
        y: -raw.z,
        z: -raw.y,
    }
}
