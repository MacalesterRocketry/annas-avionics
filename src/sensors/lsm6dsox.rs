//! LSM6DSOX: gyro + low-G accel (I²C). Skeleton.
//!
//! Target config (matches C++):
//!   - accel range  ±16 g
//!   - accel ODR    1.66 kHz
//!   - gyro  range  ±2000 °/s
//!   - gyro  ODR    1.66 kHz
//!
//! TODO: Don't inherit that 1.66 kHz gyro ODR from the C++ — the loop reads this
//!   at ~400 Hz, so sampling a 1.66 kHz stream decimates it 4x with no filter in
//!   between and folds everything from 200–830 Hz down into 0–200 Hz. That
//!   happens at the read, before AHRS sees anything, so no software filter can
//!   recover it. Instead set the gyro ODR to 416 Hz (the ladder is
//!   12.5/26/52/104/208/416/833/1660/3330/6660) and enable the on-chip LPF1.
//!   The on-chip filter runs ahead of the sensor's own decimation, so it does
//!   what software provably cannot. GYRO_LPF_HZ then only has to cover
//!   416 Hz → 50 Hz. Confirm the register details against ST's datasheet.
//! TODO: Same question for the accel ODR — pick it against the actual read rate
//!   rather than carrying over the C++ value.
//!
//! Driver strategy: use the `lsm6dsox` crate behind the `sensors` Cargo
//! feature. If its API turns out to be too sync-flavored, fall back to a
//! 200-line register-level driver — the LSM6DSOX register map is well-
//! documented in ST's datasheet and we only need a handful of writes.
//! TODO: Rotate the result to the correct orientation, since it sits in an orientation
//!  where -y is what we want Z to be.
use accelerometer::Accelerometer;
use defmt::*;
use embassy_time::Delay;
use embedded_hal::i2c::I2c;
use lsm6dsox::*;
use serde::{Deserialize, Serialize};
use crate::config::{G, LSM_ACCEL_BIAS, LSM_GYRO_BIAS};
use crate::sensors::{transform_sensor_axes, Sensor, AccelerometerReading, GyroscopeReading, TemperatureReading};
use crate::utils::math::{AngularVec3, Vec3};

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct LsmReading {
    /// Body-frame linear acceleration (m/s²), bias-subtracted.
    pub accel: Vec3,
    /// Body-frame angular rate (rad/s), bias-subtracted.
    pub gyro: AngularVec3,
    /// Die temperature (°C).
    pub temperature: f64,
}

impl AccelerometerReading for LsmReading {
    fn saturation_threshold_g(&self) -> f64 { 16.0 }
    fn accel(&self) -> Vec3 { self.accel }
}
impl GyroscopeReading for LsmReading {
    fn gyro(&self) -> AngularVec3 {
        self.gyro
    }
}
impl TemperatureReading for LsmReading {
    fn temperature(&self) -> f64 {
        self.temperature
    }
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    Init(LsmError),
    Read(LsmError),
}
#[derive(Debug, defmt::Format)]
pub enum LsmError {
    /// An error occured during a I2C write operation
    I2cWriteError,
    /// An error occured during a I2C read operation
    I2cReadError,
    /// Reset of the LSM6DSOX failed
    ResetFailed,
    /// No data was ready to be read
    NoDataReady,
    /// Invalid or unexpected data was read
    InvalidData,
    /// A parameter was incorrect
    InvalidInput,
    /// Operation or configuration not supported
    NotSupported,

    // Accelerometer-specific errors from the `accelerometer` crate
    /// Error in the underlying communications bus (e.g. I2C, SPI)
    AccelBus,
    /// Device invalid or other hardware error
    AccelDevice,
    /// Device is in an invalid mode to complete the requested operation
    AccelMode,
    /// Invalid parameter
    AccelParam,
}
type DriverError<I2C: I2c> = lsm6dsox::Error<I2C::Error>;
type AccelError<I2C: I2c> = accelerometer::Error<DriverError<I2C>>;

impl<E: embedded_hal::i2c::Error> From<&lsm6dsox::Error<E>> for LsmError {
    fn from(error: &lsm6dsox::Error<E>) -> Self {
        match error {
            lsm6dsox::Error::I2cWriteError(_) => LsmError::I2cWriteError,
            lsm6dsox::Error::I2cReadError(_) => LsmError::I2cReadError,
            lsm6dsox::Error::ResetFailed => LsmError::ResetFailed,
            lsm6dsox::Error::NoDataReady => LsmError::NoDataReady,
            lsm6dsox::Error::InvalidData => LsmError::InvalidData,
            lsm6dsox::Error::InvalidInput => LsmError::InvalidInput,
            lsm6dsox::Error::NotSupported => LsmError::NotSupported,
        }
    }
}
impl<E: embedded_hal::i2c::Error> From<lsm6dsox::Error<E>> for LsmError {
    fn from(err: lsm6dsox::Error<E>) -> Self { (&err).into() }
}

impl<E: embedded_hal::i2c::Error> From<accelerometer::Error<lsm6dsox::Error<E>>> for LsmError {
    fn from(error: accelerometer::Error<lsm6dsox::Error<E>>) -> Self {
        if let Some(cause) = error.cause() {
            return cause.into();
        }

        // Fallback to the generic error kind
        match error.kind() {
            accelerometer::ErrorKind::Bus => LsmError::AccelBus,
            accelerometer::ErrorKind::Device => LsmError::AccelDevice,
            accelerometer::ErrorKind::Mode => LsmError::AccelMode,
            accelerometer::ErrorKind::Param => LsmError::AccelParam,
        }
    }
}

trait LsmResultExt<T> {
    fn init_err(self) -> Result<T, Error>;
    fn read_err(self) -> Result<T, Error>;
}

// Blanket implementation for any Result that has an error convertable to LsmError
impl<T, E> LsmResultExt<T> for Result<T, E>
where
    E: Into<LsmError>,
{
    fn init_err(self) -> Result<T, Error> {
        self.map_err(|e| Error::Init(e.into()))
    }
    fn read_err(self) -> Result<T, Error> {
        self.map_err(|e| Error::Read(e.into()))
    }
}

pub struct Lsm<I2C: I2c> {
    driver: Lsm6dsox<I2C, Delay>,
}

impl<I2C: I2c> Sensor for Lsm<I2C> {
    type Bus = I2C;
    type Reading = LsmReading;
    type Error = Error;

    fn init(i2c: I2C) -> Result<Self, Error> {
        // TODO: Figure out what address I actually need to use
        let mut driver = Lsm6dsox::new(i2c, SlaveAddress::Low, Delay);

        driver.setup().init_err()?;
        driver.set_accel_sample_rate(DataRate::Freq416Hz).init_err()?; // TODO: tune based on loop speed
        driver.set_accel_scale(AccelerometerScale::Accel16g).init_err()?;
        driver.enable_interrupts(true).init_err()?;
        driver.map_interrupt(InterruptSource::EmbeddedFunctions, InterruptLine::INT1, true).init_err()?;
        if let Ok(reading) = driver.accel_norm() {
            info!("Acceleration: {:?}", Debug2Format(&reading));
        }
        Ok(Self { driver })
    }

    /// Read XYZ, bias-corrected to m/s².
    fn read(&mut self) -> Result<LsmReading, Error> {
        let raw_accel: Vec3 = self.driver.accel_norm().read_err()?.into();
        // The sensor is mounted in a different orientation than we want, so we need to transform the axes.
        let transformed_accel = transform_sensor_axes(raw_accel * G);
        let temperature = self.driver.temperature().read_err()?;
        let gyro: AngularVec3 = self.driver.angular_rate().read_err()?.into();
        // TODO: return None if it hasn't yet updated (DATA_READY register)
        Ok(LsmReading {
            accel: transformed_accel - LSM_ACCEL_BIAS,
            gyro: gyro - LSM_GYRO_BIAS,
            temperature: temperature.as_celsius(),
        })
    }
}
