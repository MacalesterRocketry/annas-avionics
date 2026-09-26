//! Sensor I/O stuff. Each chip has a file wrapping its driver, which provides a struct
//! with `init()` and `read()` methods that uses an I2c bus type. These structs are then
//! owned by the `Sensors` struct, and failed reads go through a debouncer that decides
//! when a chip is considered faulted.

#![allow(dead_code, unused_variables)]

pub mod adxl375;
pub mod bmp390;
pub mod health;
pub mod lis3mdl;
pub mod lsm6dsox;

use crate::sensors::health::{FaultDebouncer, SensorFault};
use crate::utils::errors::{Subsystem, SubsystemError, mark_init_complete, report_init_error};
use crate::utils::math::{AngularVec3, Vec3};
use serde::{Deserialize, Serialize};

/// Owns every sensor driver instance sharing the I²C bus, plus the debouncer
/// deciding when that chip's read failures amount to a fault.
///
/// A driver is `None` when it failed to initialize, just like if it stopped reading.
/// TODO: add lsm/lis3/bmp fields once their drivers are wired in. Maybe using `embassy_embedded_hal::shared_bus`?
pub struct Sensors<I2C: embedded_hal::i2c::I2c> {
    adxl: Option<adxl375::Adxl<I2C>>,
    adxl_health: FaultDebouncer,
}

/// A failure attributable to one chip, which carries the driver's own error so defmt can actually
/// log what went wrong.
#[derive(Debug, defmt::Format)]
pub enum SensorError {
    Adxl(adxl375::Error),
}

impl SubsystemError for SensorError {
    fn subsystem(&self) -> Subsystem {
        Subsystem::SENSORS
    }
}

/// Attempt to initialize every sensor on the shared I²C bus. Errors are marked individually,
/// but the overall sensor system can still be used even if some chips aren't working.
pub fn init_all<I2C: embedded_hal::i2c::I2c>(i2c: I2C) -> Sensors<I2C> {
    // TODO: `Adxl::init` takes the bus by value, so a failure drops it. That is
    //   harmless while the ADXL is the only chip wired in, but the second driver
    //   to land needs the bus shared (`embassy_embedded_hal::shared_bus`) so one
    //   chip's failure cannot take the other chips' bus down with it.
    let adxl = match adxl375::Adxl::init(i2c) {
        Ok(driver) => Some(driver),
        Err(e) => {
            report_init_error(SensorError::Adxl(e));
            health::mark_faulted(SensorFault::ADXL);
            None
        }
    };

    if health::faults().is_empty() {
        mark_init_complete(Subsystem::SENSORS);
    }

    Sensors {
        adxl,
        adxl_health: FaultDebouncer::new(SensorFault::ADXL),
    }
}

impl<I2C: embedded_hal::i2c::I2c> Sensors<I2C> {
    /// Read every sensor that is answering. Returns biased + axis-corrected
    /// readings, with `None` for any chip that had nothing to give.
    ///
    /// TODO: lsm/lis3/bmp read as absent until their drivers land.
    /// TODO: Make the sensors return None on stale data.
    ///   They all expose a data-ready bit (LSM6DSOX `STATUS_REG`, LIS3MDL
    ///   `STATUS_REG`, BMP390 `STATUS`), and their INT pins are already wired
    ///   through `InterruptConfig`.
    ///   Sensors run at different rates, so this is per-sensor, not per-tick.
    ///   A stale sensor is the same shape as an absent one — `None` for that
    ///   chip this tick — so it needs no new plumbing downstream.
    pub async fn read_all(&mut self) -> SensorReadings {
        let adxl = match self.adxl.as_mut() {
            Some(adxl) => self
                .adxl_health
                .record(adxl.read().map_err(SensorError::Adxl)),
            None => None,
        };
        SensorReadings {
            adxl,
            ..Default::default()
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct LsmReading {
    /// Body-frame linear acceleration (m/s²), bias-subtracted.
    pub accel: Vec3,
    /// Body-frame angular rate (rad/s), bias-subtracted.
    pub gyro: AngularVec3,
    /// Die temperature (°C).
    pub temperature: f64,
}

impl LsmReading {
    pub fn has_accel_saturated(&self) -> bool {
        self.accel.mag() >= crate::config::ACCELEROMETER_SWITCH_THRESHOLD
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct Lis3Reading {
    /// Body-frame magnetic field (µT), hard-iron corrected.
    pub mag: Vec3,
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct AdxlReading {
    /// Body-frame high-G acceleration (m/s²), bias-subtracted.
    pub accel: Vec3,
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct BmpReading {
    pub pressure: f64,    // Pa
    pub temperature: f64, // °C
    pub altitude: f64,    // m
}

/// One tick's worth of sensor data. A `None` field is a chip that didn't return any data
/// this tick, such as from an init or runtime failure or even just the sensor being
/// slower than the tick rate.
#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct SensorReadings {
    pub lsm: Option<LsmReading>,
    pub lis3: Option<Lis3Reading>,
    pub adxl: Option<AdxlReading>,
    pub bmp: Option<BmpReading>,
    launched: bool,
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
        if !self.launched && self.check_launch_accel() {
            self.launched = true;
        }
        self.launched
    }

    fn check_launch_accel(&self) -> bool {
        self.merged_accel().is_some_and(|accel| {
            accel.mag() >= crate::config::LAUNCH_ACCEL_THRESHOLD_G * crate::config::G
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
