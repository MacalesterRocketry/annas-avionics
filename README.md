# Anna's Avionics

This is the code for the avionics stack for the Macalester College High Power Rocketry team. It is currently in in active development. The Arduino version (under the `platformio` directory) is capable of active roll control, and a rewrite to Rust is underway.

For the Rust version, here's how to run it. For each, append `--release` if the code on the device is running in release mode. This includes all the "just attach" versions; the logger needs to match its lookup table to whatever is running on the device.

Build, flash, and attach: `cargo embed`

Reset the device and attach: `cargo embed reset_attach`

Just attach: `cargo embed attach`

New hardware plans:
- Custom PCB containing everything
- Microcontroller: RP2350B
- High-G accelerometer: ADXL375
- Gyroscope and low-G accelerometer: LSM6DSOX
- Magnetometer: Considering either the MMC5983 or LIS3MDL, leaning towards the former
- Barometer: BMP390
- GPS module: Probably ublox MAX-M10S

Hardware (old version):
- Microcontroller: Adafruit Feather RP2040 Adalogger
- High-G accelerometer: ADXL375
- Gyroscope and low-G accelerometer: LSM6DSOX
- Magnetometer: LIS3MDL
- Barometer: BMP390
- GPS module: MTK3333 on an Adafruit Ultimate GPS Featherwing

At the moment, all sensors but the GPS are connected over I²C using Stemma QT, and the GPS module is connected over UART. A custom FeatherWing PCB is in progress to combine all the sensors onto one board, improve interrupt support, add battery monitoring, and much more.

Supports a buzzer with built-in driver circuitry for state notification; PWM buzzers are not supported at the moment.

## Log decoder

Use `decoder.py` to decode a `logNN.bin` file to CSV and optionally re-export a filtered binary by timestamp (`micros`) range.

```bash
python decoder.py --search-dir /path/to/sd_dump --start-micros 1200000 --end-micros 15200000 --output-bin ignition_to_landing.bin --output-csv ignition_to_landing.csv
```

Notes:
- Range bounds are inclusive (`start <= micros <= end`).
- If `--input-bin` is omitted, the script selects the latest `logNN.bin` in `--search-dir`.
- The filtered `.bin` keeps the original file header and packet bytes so it can be decoded again later.

