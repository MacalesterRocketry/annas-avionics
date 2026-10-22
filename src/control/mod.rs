//! Attitude control.
//!
//! One task at the servo frame rate owns the whole chain: read the latest AHRS
//! solution and setpoint, run the PID, mix to fin deflections, write the servos.
//!
//! It deliberately does *not* run at the sensor rate. The PWM frame is 20 ms and
//! the servos themselves respond at roughly 10 Hz, so 50 Hz already oversamples
//! the actuator by ~10x and a faster compensator buys no closed-loop
//! performance. Splitting PID and servo writes into separate tasks would be
//! worse still: it adds up to another full period of actuation delay, and at a
//! ~5 Hz bandwidth 20 ms is already ~36° of phase. Keeping it in one task also
//! means the PID error and the effectiveness lookup come from the same AHRS
//! snapshot rather than two different instants.
//!
//! The three layers stay separate as modules, not as tasks: [`pid`] is the
//! control law, [`fins`] the effector model, [`servo`] the hardware.

pub mod fins;
pub mod pid;
pub mod servo;

use core::sync::atomic::{AtomicBool, Ordering};
use crate::config::board::ServoConfig;
use crate::control::pid::RollPid;
use crate::control::servo::init_servos;
use crate::navigation::ahrs::AHRS_STATE;
use crate::utils::errors::{Subsystem, SubsystemError, clear_runtime_error, mark_init_complete, report_init_error, report_runtime_error};
use defmt::{Debug2Format, Format, info};
use embassy_rp::pwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use serde::{Deserialize, Serialize};
use crate::utils::math::Quat;

/// Everything that can go wrong in the attitude control subsystem.
///
/// `pwm::PwmError` is embassy's and only derives `Debug`, so [`Format`] is
/// hand-written here to route it through `Debug2Format`.
#[derive(Debug)]
pub enum ControlError {
    /// The servo PWM slices could not be configured at startup.
    ServoInit(pwm::PwmError),
    /// Writing a deflection command to a servo failed.
    ServoWrite(pwm::PwmError),
    /// Too many `AHRS_STATE` receivers are in use, so another can't be created.
    NoAhrsReceiver,
    /// Too many `CONTROL_SETPOINT` receivers are in use, so another can't be created.
    NoSetpointReceiver,
}

impl Format for ControlError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::ServoInit(e) => defmt::write!(f, "servo init failed: {:?}", Debug2Format(e)),
            Self::ServoWrite(e) => defmt::write!(f, "servo write failed: {:?}", Debug2Format(e)),
            Self::NoAhrsReceiver => {
                defmt::write!(f, "no AHRS receiver available; too many initialized?")
            }
            Self::NoSetpointReceiver => {
                defmt::write!(f, "no control setpoint receiver available; too many initialized?")
            }
        }
    }
}

impl SubsystemError for ControlError {
    fn subsystem(&self) -> Subsystem {
        Subsystem::CONTROL
    }
}

/// What the control loop should be doing. The state machine owns the mission
/// timeline, so it decides the attitude to hold; the control loop only knows how
/// to get there.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub enum ControlSetpoint {
    /// Fins centered, PID held in reset.
    Neutral,
    /// Drive the airframe to this absolute attitude.
    Attitude(Quat),
}

/// Latest control setpoint. A `Watch` because the control loop wants whatever is
/// current when its own ticker fires — a queue would let setpoints pile up and
/// be actuated late.
static CONTROL_SETPOINT: Watch<CriticalSectionRawMutex, ControlSetpoint, 2> = Watch::new();
static CONTROL_ARMED: AtomicBool = AtomicBool::new(false);

pub fn is_control_armed() -> bool {
    CONTROL_ARMED.load(Ordering::Acquire)
}

/// Disarms the control loop, such as in case of a fault. Control setpoints do nothing when disarmed.
pub fn disarm_control() {
    CONTROL_ARMED.store(false, Ordering::Release);
    CONTROL_SETPOINT.sender().send(ControlSetpoint::Neutral);
}
pub fn arm_control() {
    CONTROL_ARMED.store(true, Ordering::Release);
}

pub fn set_control_setpoint(setpoint: ControlSetpoint) {
    if is_control_armed() {
        CONTROL_SETPOINT.sender().send(setpoint);
    }
}

pub async fn control_loop(servo_config: ServoConfig) {
    info!("initializing servos");
    let mut servos = match init_servos(servo_config) {
        Ok(servos) => {
            info!("servos initialized");
            mark_init_complete(Subsystem::CONTROL);
            servos
        }
        Err(e) => {
            report_init_error(ControlError::ServoInit(e));
            return;
        }
    };

    let Some(mut ahrs_rx) = AHRS_STATE.receiver() else {
        report_init_error(ControlError::NoAhrsReceiver);
        return;
    };
    let Some(mut setpoint_rx) = CONTROL_SETPOINT.receiver() else {
        report_init_error(ControlError::NoSetpointReceiver);
        return;
    };

    let mut pid = RollPid::new();
    let mut armed = false;

    let mut ticker = Ticker::every(Duration::from_hz(servo::SERVO_PWM_HZ as u64));
    loop {
        ticker.next().await;

        // `try_get` rather than `changed`: the ticker decides when to actuate,
        // and the setpoint is whatever is current at that moment. Treat a
        // missing setpoint or AHRS solution as disarmed — before the first
        // sensor sample there is nothing to control against.
        let setpoint = setpoint_rx.try_get().unwrap_or(ControlSetpoint::Neutral);
        let ahrs = ahrs_rx.try_get();

        let command = match (setpoint, ahrs) {
            (ControlSetpoint::Attitude(qtarget), Some(ahrs)) => {
                if !armed {
                    // Drop any integral accumulated before this arming.
                    pid.reset();
                    armed = true;
                    info!("control armed");
                }
                Some((pid.step(qtarget, &ahrs), ahrs.get_velocity_earth()))
            }
            _ => {
                if armed {
                    armed = false;
                    info!("control disarmed");
                }
                None
            }
        };

        let result = match command {
            Some((ang_accel, velocity_earth)) => {
                fins::apply(&mut servos, velocity_earth, ang_accel)
            }
            None => servos.center_all(),
        };
        match result {
            Err(e) => report_runtime_error(ControlError::ServoWrite(e)),
            // `clear_runtime_error` only logs if the bit was actually set, so
            // this is cheap to call on every successful frame.
            Ok(()) => clear_runtime_error(Subsystem::CONTROL),
        }
    }
}