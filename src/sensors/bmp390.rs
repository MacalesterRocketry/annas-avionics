use bmp390::{Bmp390, ConfigurationBuilder};
use bmp390::interfaces::{I2cInterface, Polling};
use bmp390::registers::*;
use defmt::info;
use crate::sensors::{AltitudeReading, PressureReading, Sensor, TemperatureReading};
use embassy_rp::gpio::Input;
use embassy_time::Delay;
use embedded_hal::i2c::I2c;
use serde::{Deserialize, Serialize};

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct BmpReading {
    pub pressure: f32,    // Pa
    pub temperature: f32, // °C
    pub altitude: f32,    // m
}

impl PressureReading for BmpReading {
    fn pressure(&self) -> f32 {
        self.pressure
    }
}

impl AltitudeReading for BmpReading {
    fn altitude(&self) -> f32 {
        self.altitude
    }
}

impl TemperatureReading for BmpReading {
    fn temperature(&self) -> f32 {
        self.temperature
    }
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    Init,
    Read,
}

pub struct Bmp<I2C: I2c> {
    driver: bmp390::Bmp390<Polling<I2cInterface<I2C>, Delay>>,
}

impl<I2C: I2c> Sensor for Bmp<I2C> {
    type Bus = I2C;
    type Reading = BmpReading;
    type Error = Error;

    async fn read(&mut self) -> Result<Self::Reading, Self::Error>
        where
            Self: Sized
    {
        let measurement = self.driver.measure().map_err(|_| Error::Read)?; // TODO: handle error better
        // TODO: wait for interrupt
        Ok(BmpReading {
            // TODO: use uom throughout everything
            pressure: measurement.pressure.get::<uom::si::pressure::pascal>(),
            temperature: measurement.temperature.get::<uom::si::thermodynamic_temperature::degree_celsius>(),
            altitude: measurement.altitude.get::<uom::si::length::meter>(),
        })
    }
}

impl<I2C: I2c> Bmp<I2C> {
    pub fn init(i2c: I2C, int_pin: Input<'static>) -> Result<Self, Error>
        where
            Self: Sized
    {
        let interface = Polling {
            interface: I2cInterface {
                bus: i2c,
                address: bmp390::interfaces::Sdo::Down, // TODO: figure out what this is actually set to
            },
            delay: Delay,
        };
        let mut driver = Bmp390::new(interface);
        let chip_id = driver.device().chip_id().read().map_err(|_| Error::Init)?;
        info!("Chip ID: {:#04X}", chip_id.value());
        let rev_id = driver.device().rev_id().read().map_err(|_| Error::Init)?;
        info!("Rev ID: {:#04X}", rev_id.value());

        // read event and interrupt status registers to clear any pending interrupts
        driver.device().event().read().map_err(|_| Error::Init)?;
        driver.device().int_status().read().map_err(|_| Error::Init)?;

        // configure
        let builder = ConfigurationBuilder::new()
            .iir_filter(FilterCoefficient::Coefficient3)
            .output_data_rate(OutputDataRate::Hz100)
            .oversampling(OversamplingConfig {
                pressure: Oversampling::X8,
                temperature: Oversampling::X4,
            })
            .power_control(PowerControl {
                pressure_enabled: true,
                temperature_enabled: true,
                mode: PowerMode::Normal,
            });

        driver.configure(builder).map_err(|_| Error::Init)?; // TODO: Can this error be handled better?

        // check for errors after writing config
        let err_reg = driver.device().err_reg().read().map_err(|_| Error::Init)?;
        if err_reg.conf_err() {
            return Err(Error::Init);
        }

        let result = driver.execute(Command::SoftReset).map_err(|_| Error::Init)?;

        info!("Soft reset: {:?}", result);

        let measurement = driver.measure().map_err(|_| Error::Init)?; // TODO: Can this error be handled better?
        // sets the altitude so we're going from AGL, not MSL
        driver.set_reference_altitude(measurement.altitude);

        Ok(Self { driver })
    }
}