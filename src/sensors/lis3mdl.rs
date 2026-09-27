//! LIS3MDL magnetometer (I²C). Skeleton.
//!
//! Target config:
//!   - operation mode  continuous
//!   - data rate       560 Hz
//!   - range           ±4 gauss
//!   - performance     high

use serde::{Deserialize, Serialize};
use crate::sensors::MagnetometerReading;
use crate::utils::math::Vec3;

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct Lis3Reading {
    /// Body-frame magnetic field (µT), hard-iron corrected.
    pub mag: Vec3,
}

impl MagnetometerReading for Lis3Reading {
    fn mag(&self) -> Vec3 {
        self.mag
    }
}