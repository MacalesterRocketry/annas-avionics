//! BMP390 barometer (I²C). Skeleton.
//!
//! Target config (matches BMP3XX usage in C++):
//!   - temperature oversampling  8×
//!   - pressure oversampling     4×
//!   - IIR filter coeff          3
//!   - output data rate          100 Hz
//!
//! Driver strategy: the `bmp388` crate works for the BMP390 (registers are
//! identical between BMP388 and BMP390; only the pressure range differs).
//! Altitude is computed from pressure with the standard barometric formula
//! using SEALEVELPRESSURE_HPA = 1013.25 — same constant as the C++ source.

use serde::{Deserialize, Serialize};
use crate::sensors::{AltitudeReading, PressureReading, TemperatureReading};

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct BmpReading {
    pub pressure: f64,    // Pa
    pub temperature: f64, // °C
    pub altitude: f64,    // m
}

impl PressureReading for BmpReading {
    fn pressure(&self) -> f64 {
        self.pressure
    }
}

impl AltitudeReading for BmpReading {
    fn altitude(&self) -> f64 {
        self.altitude
    }
}

impl TemperatureReading for BmpReading {
    fn temperature(&self) -> f64 {
        self.temperature
    }
}