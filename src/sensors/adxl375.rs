//! ADXL375 ±200 g accelerometer (I²C). Wraps the `adxl3xx` driver crate.
//!
//! Scale factor: 49 mg/LSB on ADXL375 (per datasheet table 1).
//!
//! TODO: `init_defaults` leaves this at the datasheet default 800 Hz ODR while
//!   the loop reads it at ~400 Hz, so content above 200 Hz aliases down into the
//!   band at the read and cannot be filtered out afterward. The ADXL375's
//!   internal anti-alias filter is tied to the ODR (roughly ODR/4), so pick the
//!   ODR deliberately against the read rate instead of taking the default —
//!   same reasoning as the gyro TODO in `lsm6dsox.rs`. Launch detection only
//!   needs a magnitude spike so it is tolerant, but `merged_accel` feeds AHRS
//!   once the low-G accel saturates, and that path is not.

use adxl3xx::{Adxl375 as Adxl3xxDriver, AdxlBusI2c};
use embassy_rp::gpio::Input;
use embedded_hal::i2c::I2c;
use serde::{Deserialize, Serialize};
use crate::config::{G, ADXL_BIAS};
use crate::sensors::{transform_sensor_axes, Sensor, AccelerometerReading};
use crate::utils::math::Vec3;

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct AdxlReading {
    /// Body-frame high-G acceleration (m/s²), bias-subtracted.
    pub accel: Vec3,
}

impl AccelerometerReading for AdxlReading {
    fn saturation_threshold_g(&self) -> f64 { 200.0 }
    fn accel(&self) -> Vec3 { self.accel }
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    /// Device ID mismatch or register I/O failure during setup.
    Init,
    /// Axis-offset calibration failed.
    Calibration,
    /// Axis read failed.
    Read,
}

pub struct Adxl<I2C: I2c> {
    driver: Adxl3xxDriver<AdxlBusI2c<I2C>>,
    int1: Input<'static>,
    int2: Input<'static>,
}

impl<I2C: I2c> Sensor for Adxl<I2C> {
    type Bus = I2C;
    type Reading = AdxlReading;
    type Error = Error;

    /// Read XYZ, bias-corrected to m/s².
    async fn read(&mut self) -> Result<AdxlReading, Error> {
        self.int1.wait_for_rising_edge().await; // TODO: Decide if we want this.
        let raw: Vec3 = self.driver.read_axis().map_err(|_| Error::Read)?.into();
        // The sensor is mounted in a different orientation than we want, so we need to transform the axes.
        let transformed = transform_sensor_axes(raw * G);
        Ok(AdxlReading { // TODO: Decide if I want to have optional returns or async returns. Leaning toward the latter, but unsure. Will need work either way.
            accel: transformed - ADXL_BIAS,
        })
    }
}

impl<I2C: I2c> Adxl<I2C> {
    /// Bring up the high-G accelerometer: validate device ID, reset to
    /// datasheet defaults (800 Hz, FIFO stream), and run axis-offset
    /// calibration. Mirrors `initHighGAccelerometer()` in the C++ source,
    /// minus the activity-interrupt wiring — launch detection currently
    /// polls magnitude instead (see `SensorReadings::has_launched`).
    pub(crate) fn init(i2c: I2C, int_pin1: Input<'static>, int_pin2: Input<'static>) -> Result<Self, Error> where Self: Sized {
        let bus = AdxlBusI2c { i2c, addr: adxl3xx::reg::ADXL_ADDR };
        let mut driver = Adxl3xxDriver::new(bus).map_err(|_| Error::Init)?;

        driver.init_defaults().map_err(|_| Error::Init)?;
        // TODO: figure out if we want to auto calibrate on launchpad somehow
        // driver.calibrate_axis_offsets().map_err(|_| Error::Calibration)?;
        // TODO: set up DATA_READY interrupt

        Ok(Self {
            driver,
            int1: int_pin1,
            int2: int_pin2,
        })
    }
}