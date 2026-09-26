//! Compile-time configuration. Direct port of the C++ `config.h`.
//!
//! Pin names use generic `PIN_*` labels keyed by GPIO number so the user can
//! remap them when the RP2350 PCB layout is finalized. The hardware-revision
//! gate from the C++ source becomes a Cargo feature (`hw-v2` / `hw-v3`).

#![allow(dead_code)]

use crate::utils::math::AngularVec3;

// ─────────────────────────────── debug knobs ────────────────────────────────
// In the C++ these are `#define X 0/1` preprocessor flags. Here they're plain
// `const bool` — the optimizer constant-folds them just like the preprocessor,
// and they're typesafe.
pub const DEBUG: bool = false;
pub const DEBUG_SD: bool = false;
pub const DEBUG_PRINT_SENSORS: bool = false;
pub const DEBUG_PRINT_ORIENTATION: bool = false;
pub const DEBUG_PRINT_ROLL_CONTROL: bool = false;

pub const PID_TUNING: bool = false;
pub const SERVO_TESTING: bool = false;

pub const PROFILING: bool = false;
pub const PROFILING_SAMPLES: usize = 100;

pub const SYNC_INTERVAL_MS: u64 = 5_000;

pub const USE_GPS: bool = true;
pub const USE_TURN_SIGNALS: bool = true;
pub const HAS_DROGUE_CHUTE: bool = true;

// ───────────────────────────── physical constants ───────────────────────────
/// North Branch, MN local gravity (m/s²).
pub const G: f64 = 9.805_79;

// Magnetometer hard-iron offsets (µT). Calibrate per-flight.
pub const MAG_BIAS_X: f64 = -35.63;
pub const MAG_BIAS_Y: f64 = 19.96;
pub const MAG_BIAS_Z: f64 = -33.43;

// High-G (ADXL375) zero offsets. NOTE from C++ source: the ADXL375 may apply
// internal gravity correction — verify on bench before trusting these.
pub const HIGHG_BIAS_X: f64 = 9.8; // TODO: implement this, I don't think it's currently used
pub const HIGHG_BIAS_Y: f64 = 7.85;
pub const HIGHG_BIAS_Z: f64 = 9.8 - G;
pub const HIGHG_TRIM_X: i8 = -5; // raw LSB counts, not m/s²
pub const HIGHG_TRIM_Y: i8 = -4;
pub const HIGHG_TRIM_Z: i8 = 0;

// Low-G (LSM6DSOX accel) zero offsets.
pub const LOWG_BIAS_X: f64 = 0.05;
pub const LOWG_BIAS_Y: f64 = 0.03;
pub const LOWG_BIAS_Z: f64 = 10.03 - G;

// Gyro zero-rate offsets (rad/s).
pub const GYRO_BIAS_X: f64 = -0.01;
pub const GYRO_BIAS_Y: f64 = 0.0;
pub const GYRO_BIAS_Z: f64 = 0.0;

// ───────────────────────────── AHRS constants ───────────────────────────────
/// Madgwick β for accelerometer correction. Higher → trust accel more, faster
/// convergence, more noise.
pub const AHRS_ACC_BETA: f64 = 0.1;
/// Madgwick β for magnetometer correction. 0.0 disables it (current default).
pub const AHRS_MAG_BETA: f64 = 0.0;

/// Cutoff frequency (Hz) of the gyro low-pass that feeds the PID's D term.
///
/// This is an anti-aliasing filter. The control loop samples AHRS at
/// [`control::servo::SERVO_PWM_HZ`] (50 Hz), whose Nyquist frequency is
/// 25 Hz, so gyroscope responses above 25 Hz become indistinguishable from
/// the lower-frequency information we actually care about. 190 Hz noise,
/// for instance, lands at |190 − 4·50| = 10 Hz.
/// 
/// This needs to be below 25 Hz or it won't actually filter out the noise,
/// and faster is better to reduce latency and therefore lag.
pub const GYRO_LPF_HZ: f64 = 20.0;

// ────────────────────────────── PID constants ───────────────────────────────
/// Roll-axis moment of inertia (kg·m²).
pub const MOMENT_OF_INERTIA: f64 = 0.006_85;
/// CFD-derived torque at 10° fin deflection, 50 m/s airspeed (N·m).
pub const BASE_TORQUE: f64 = 0.429_709_735_258_359;
/// Linear approximation of torque per degree at 50 m/s (N·m/deg).
pub const TORQUE_PER_DEG_50MS: f64 = BASE_TORQUE / 10.0;
pub const ROLL_PID_KP: f64 = 68.1295;
pub const ROLL_PID_KI: f64 = 2.2519;
pub const ROLL_PID_KD: f64 = 20.8804;

// ─────────────────────────────── thresholds ────────────────────────────────
pub const LAUNCH_ACCEL_THRESHOLD_G: f64 = 4.0;
/// Mechanical range of the servo in degrees when unrestricted by the airframe.
pub const SERVO_DEGREE_RANGE: f64 = 100.0;
/// These represent the physical limits of the servo where it is currently placed and are relative to inline with the fin.
pub const SERVO_MAX_ANGLE: f64 = 50.0;
pub const SERVO_MIN_ANGLE: f64 = -50.0;
/// These represent the logical limits of what control signals can be sent to the servo.
pub const SERVO_MICROS_MIN: u16 = 1000;
pub const SERVO_MICROS_MAX: u16 = 2000;

/// Per-servo mechanical zero offset in degrees.
/// Shifts the whole window, so one end of the range ends up with a few degrees cut off.
pub const SERVO_TRIM: board::Trims = board::Trims {
    xplus: -3.0,
    xminus: -3.0,
    yplus: -3.0,
    yminus: -3.0,
};

/// Inverts the direction of servos for if they're mounted backwards
pub const SERVO_INVERT: board::Inverts = board::Inverts {
    xplus: false,
    xminus: false,
    yplus: false,
    yminus: false,
};
/// Datasheet stall torque at 7.4 V (N·m).
pub const SERVO_MAX_TORQUE: f64 = 0.51;

/// Each fin's contribution to each control axis, from -1 to 1.
///
/// Pitch authority is proportional to cos(theta) where theta is the angle
/// relative to some shared axis, while yaw is proportional to sin(theta).
/// For example, for three fins, one has a yaw authority factor of 1.0 and pitch
/// of 0.0, while the other two have -1/2 for yaw and ±sqrt(3)/2 for pitch.
/// 
/// Basically, just put your fins as points on a unit circle. The distance
/// from the y axis to each fin is the yaw factor, while the distance from
/// the x axis is the pitch factor. Roll is always 1 unless you're doing
/// something really weird.
pub const FIN_MIX: board::FinMix = board::FinMix {
    xplus: AngularVec3 { pitch: 0.0, yaw: 0.0, roll: 1.0 },
    xminus: AngularVec3 { pitch: 0.0, yaw: 0.0, roll: 1.0 },
    yplus: AngularVec3 { pitch: 0.0, yaw: 0.0, roll: 1.0 },
    yminus: AngularVec3 { pitch: 0.0, yaw: 0.0, roll: 1.0 },
};
/// LSM6DSOX low-G accel saturates at 16 g; switch to ADXL375 above this.
pub const ACCELEROMETER_SWITCH_THRESHOLD: f64 = 15.9 * G;

// ──────────────────────────── battery divider ───────────────────────────────
pub const BATTERY_VOLTAGE_R1: f64 = 100_000.0;
pub const BATTERY_VOLTAGE_R2: f64 = 100_000.0;

#[cfg(feature = "hw-v3")]
pub mod board {
    use crate::define_hardware;
    use embassy_rp::peripherals::*;
    use embassy_rp::pio_programs::ws2812::PioWs2812;

    /// External high-speed crystal on the Metro RP2350 board is 12 MHz, as with most RP2350 boards
    pub(crate) const XTAL_FREQ_HZ: u32 = 12_000_000u32;
    /// System clock after `embassy_rp::init(Default::default())`. Anything
    /// deriving a peripheral clock divider (e.g. servo PWM) keys off this.
    pub(crate) const SYS_CLK_HZ: u32 = 150_000_000;
    pub(crate) const NUM_LEDS: usize = 1;
    pub type NeopixelColorOrder = embassy_rp::pio_programs::ws2812::Grb;
    pub type Neopixel = PioWs2812<'static, PIO0, 0, NUM_LEDS, NeopixelColorOrder>;

    // Now you map logical names to physical pins exactly once.
    // To add a pin, just add one line here.
    define_hardware!(AvionicsHardware {
        i2c: I2cConfig {
            bus: I2C0,
            sda: PIN_20,
            scl: PIN_21,
        },
        gps: GpsConfig {
            rx: PIN_1,
            tx: PIN_0,
            bus: UART0,
        },
        telemetry: TelemetryConfig {
            rx: PIN_5,
            tx: PIN_4,
            bus: UART1,
        },
        sd: SdConfig {
            spi: SPI0,
            sclk: PIN_34,
            mosi: PIN_35,
            miso: PIN_36,
            data1: PIN_37,
            data2: PIN_38,
            cs: PIN_39,
            detect: PIN_40,
        },
        interrupts: InterruptConfig {
            adxl_int1: PIN_7,
            adxl_int2: PIN_8,
            lsm_int1: PIN_9,
            lsm_int2: PIN_10,
            lis3_int1: PIN_11,
            lis3_int2: PIN_12,
            bmp_int: PIN_13,
        },
        indicators: IndicatorsConfig {
            buzzer: PIN_43, // Note: Any changes to this should also be changed in the panic handler, TODO: maybe make a constant
            neopixel: PIN_25,
            neopixel_pio: PIO0,
            neopixel_dma: DMA_CH0,
        },
        peripherals: PeripheralConfig {
            eject_button: PIN_24,
        },
    }
    servos servos: ServoConfig -> Servos {
        x_slice: PWM_SLICE5 { a: xplus = PIN_26, b: xminus = PIN_27 },
        y_slice: PWM_SLICE6 { a: yplus = PIN_28, b: yminus = PIN_29 },
    });
}

#[cfg(feature = "hw-v2")]
pub mod board {
    /// Adafruit Feather RP2040 Adalogger's external high-speed crystal is 12 MHz
    pub const XTAL_FREQ_HZ: u32 = 12_000_000;
    /// RP2040 default system clock.
    pub(crate) const SYS_CLK_HZ: u32 = 125_000_000;
    pub(crate) const NUM_LEDS: usize = 1;
    pub type NeopixelColorOrder = embassy_rp::pio_programs::ws2812::Grb;

    use embassy_rp::Peri;
    use embassy_rp::peripherals::*;

    define_hardware!(AvionicsHardware {
        i2c: I2cConfig {
            bus: I2C0,
            sda: PIN_2,
            scl: PIN_3,
        },
        uart: UartConfig {
            rx: PIN_1,
            tx: PIN_0,
        },
        sd: SdConfig {
            sclk: PIN_18,
            mosi: PIN_19,
            miso: PIN_20,
            data1: PIN_21,
            data2: PIN_22,
            cs: PIN_23,
            detect: PIN_16,
        },
        interrupts: InterruptConfig {
            adxl_int1: PIN_5,
            adxl_int2: PIN_6,
            lsm_int1: PIN_9,
            lsm_int2: PIN_10,
            lis3_int1: PIN_11,
            lis3_int2: PIN_12,
            bmp_int: PIN_13,
        },
        peripherals: PeripheralConfig {
            neopixel: PIN_4,
            buzzer: PIN_28,
            eject_button: PIN_24,
            battery_voltage: PIN_29,
            turn_signal_left: PIN_26,
            turn_signal_right: PIN_27,
        },
    }
    // Unlike v3, v2's servo pins do not pair up by axis: GPIO 8/25 share slice
    // 4 and GPIO 14/15 share slice 7, so each slice straddles the X and Y
    // pairs. Harmless — the fins are addressed by name, not by slice — but it
    // is why the slice fields are named after the slice here. Untested: this
    // board is RP2040 and the crate currently builds for rp235xb.
    servos servos: ServoConfig -> Servos {
        slice4: PWM_SLICE4 { a: yminus = PIN_8,  b: xplus = PIN_25 },
        slice7: PWM_SLICE7 { a: xminus = PIN_14, b: yplus = PIN_15 },
    });
}
