//! Central state machine. This keeps track of the overall state of the system, e.g. if it's
//! on the ground or in flight and if it's in the burn, coast, drogue descent, etc. phase of the
//! flight. If a system needs to know the state of the flight, it should either depend on this
//! or be controlled by this.

use crate::communication::indication::{BeepCycle, LedColor, StateIndicator};
use crate::config::HAS_DROGUE_CHUTE;
use crate::config::board::PeripheralConfig;
use crate::navigation::ahrs::{AhrsMode, AhrsState, AHRS_STATE};
use crate::navigation::gps::{GPS_STATE, GpsState};
use crate::utils::errors::{Subsystem, SubsystemError, is_init_settled, mark_init_complete, report_init_error};
use crate::utils::math::{Deg, Quat, roll_deg_to_quat};
use crate::FLIGHT_STATE;
use defmt::{error, info};
use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Instant, Ticker};
use serde::{Deserialize, Serialize};
pub(crate) use crate::control::{disarm_control, set_control_setpoint, ControlSetpoint, arm_control};
use crate::sensors::lsm6dsox::LAUNCH_DETECTED;

pub struct SystemState<'a> {
    pub state: FlightState,
    pub ahrs: Option<Receiver<'a, CriticalSectionRawMutex, AhrsState, 2>>,
    pub gps: Option<Receiver<'a, CriticalSectionRawMutex, GpsState, 3>>,
    pub(crate) ignition_time: Option<Instant>,
    pub(crate) launch_orientation: Quat,
    pub(crate) last_tick: Instant,
}
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub enum FlightState {
    PreLaunch(GroundSubState),
    Ascent(AscentSubState),
    Descent(DescentSubState),
    Landed,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub enum GroundSubState {
    Startup,
    ReadyToLaunch(ReadyToLaunchSubState),
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub enum ReadyToLaunchSubState {
    WaitingForGPS,
    GPSLock,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub enum AscentSubState {
    Burn,
    Coast,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub enum DescentSubState {
    DrogueDeploy,
    MainDeploy,
}

fn standard_beep_cycle<'a>(count: u32, cycle_secs: u64) -> BeepCycle<'a> {
    BeepCycle::Pulse {
        count,
        on_time: Duration::from_millis(100),
        off_time: Duration::from_millis(100),
        cycle_duration: Duration::from_secs(cycle_secs),
    }
}

impl Default for FlightState {
    fn default() -> Self {
        FlightState::PreLaunch(GroundSubState::Startup)
    }
}

impl FlightState {
    /// Returns instructions for the LED and buzzer to indicate the current state.
    pub fn indicator(&self) -> StateIndicator {
        // TODO: should probably add GPS lock check here
        match &self {
            FlightState::PreLaunch(GroundSubState::Startup) => StateIndicator {
                led: LedColor::Blue,
                buzzer: BeepCycle::Silent,
            },
            FlightState::PreLaunch(GroundSubState::ReadyToLaunch(gps_state)) => match gps_state {
                ReadyToLaunchSubState::WaitingForGPS => StateIndicator {
                    led: LedColor::Green,
                    buzzer: standard_beep_cycle(1, 8), // TODO: should probably eventually rethink this (e.g. 2 beeps if GPS lock), but fine for now
                },
                ReadyToLaunchSubState::GPSLock => StateIndicator {
                    led: LedColor::Green,
                    buzzer: standard_beep_cycle(1, 16),
                },
            },
            FlightState::Ascent(AscentSubState::Burn) => StateIndicator {
                led: LedColor::Purple,
                buzzer: BeepCycle::Silent,
            },
            FlightState::Descent(_) => StateIndicator {
                led: LedColor::Cyan,
                buzzer: BeepCycle::Silent,
            },
            FlightState::Landed => StateIndicator {
                led: LedColor::Cyan,
                buzzer: BeepCycle::Silent,
            },
            _ => StateIndicator {
                led: LedColor::Off,
                buzzer: BeepCycle::Silent,
            },
        }
    }

    pub fn ahrs_mode(&self) -> AhrsMode {
        match self {
            FlightState::PreLaunch(_) | FlightState::Landed => {
                AhrsMode::Ground
            }
            FlightState::Ascent(_) | FlightState::Descent(_) => AhrsMode::Flight,
        }
    }
}

impl<'a> SystemState<'a> {
    pub fn new() -> Self {
        let mut state = FlightState::default();
        FLIGHT_STATE.sender().send(state); // set the state machine to PreLaunch(Startup)

        Self {
            state,
            gps: GPS_STATE.receiver(),
            ahrs: AHRS_STATE.receiver(),
            ignition_time: None,
            launch_orientation: Quat::IDENTITY,
            last_tick: Instant::now(),
        }
    }

    /// The actual core state machine; this is called each tick and handles the state transitions and actions.
    pub async fn handle_state(&mut self) {
        let now = Instant::now();
        let tick_time = now - self.last_tick;
        self.last_tick = now;

        match self.state {
            // Everything that occurs on the ground prior to launch.
            FlightState::PreLaunch(sub) => {
                set_control_setpoint(ControlSetpoint::Neutral);
                match sub {
                GroundSubState::Startup => {
                    if is_init_settled() { // TODO: AHRS should probably signal if it's ready too
                        self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch(ReadyToLaunchSubState::WaitingForGPS)));
                    }
                }
                GroundSubState::ReadyToLaunch(gps_state) => {
                    if LAUNCH_DETECTED.signaled() {
                        self.transition_to(FlightState::Ascent(AscentSubState::Burn));
                    }
                    let has_fix = match self.gps.as_mut().map(|mut gps_recv| gps_recv.try_get()) {
                        Some(Some(gps)) => gps.has_fix,
                        _ => false,
                    };
                    match gps_state {
                        ReadyToLaunchSubState::WaitingForGPS => {
                            if has_fix {
                                self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch(ReadyToLaunchSubState::GPSLock)));
                            }
                        },
                        ReadyToLaunchSubState::GPSLock => {
                            if !has_fix {
                                self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch(ReadyToLaunchSubState::WaitingForGPS)));
                            }
                        }
                    }
                }
            }},

            FlightState::Ascent(sub) => {
                if self.ignition_time.is_none() { // shouldn't ever happen because of our transition function, but just in case
                    self.handle_ignition();
                }
                let time_since_ignition = now - self.ignition_time.unwrap_or(now);
                set_control_setpoint(ControlSetpoint::Attitude(
                    attitude_target(self.launch_orientation, roll_program(time_since_ignition)),
                ));
                let ahrs_opt = match self.ahrs.as_mut().map(|mut ahrs| ahrs.try_get()) {
                    Some(Some(ahrs)) => Some(ahrs),
                    _ => None,
                };
                match sub {
                    AscentSubState::Burn => {
                        // TODO: test;
                        // if acceleration is negative in z and it's already off the rail (velocity is somewhat high), switch to Coast
                        if let Some(ahrs) = ahrs_opt && ahrs.acceleration_earth.z < -0.01 && ahrs.velocity_earth.z > 20.0 {
                            self.transition_to(FlightState::Ascent(AscentSubState::Coast));
                        }
                    }
                    AscentSubState::Coast => {
                        // TODO: is apogee enough? do I need to do something more precise to detect drogue?
                        if let Some(ahrs) = ahrs_opt && ahrs.velocity_earth.z < 0.0 { // Apogee detected
                            if HAS_DROGUE_CHUTE { // TODO: How does Rust do things like this? Features?
                                self.transition_to(FlightState::Descent(DescentSubState::DrogueDeploy));
                            } else {
                                self.transition_to(FlightState::Descent(DescentSubState::MainDeploy))
                            }
                        }
                    }
                }
            }

            FlightState::Descent(sub) => {
                set_control_setpoint(ControlSetpoint::Neutral);
                match sub {
                    DescentSubState::DrogueDeploy => {}
                    DescentSubState::MainDeploy => {}
                }
            }
            FlightState::Landed => {
                set_control_setpoint(ControlSetpoint::Neutral);
                // TODO: eject SD here
            }
        }
    }

    fn transition_to(&mut self, next_state: FlightState) {
        // TODO: log
        info!("transitioning to state {:?}", next_state);
        match next_state {
            FlightState::PreLaunch(GroundSubState::ReadyToLaunch(_)) => {
                LAUNCH_DETECTED.reset()
            }
            FlightState::Ascent(AscentSubState::Burn) => {
                self.handle_ignition();
            }
            _ => {}
        }

        self.state = next_state;
        FLIGHT_STATE.sender().send(next_state);
    }

    fn handle_ignition(&mut self) {
        match self.ahrs.as_mut().map(|mut ahrs| ahrs.try_get()) {
            Some(Some(ahrs)) => {
                arm_control();
                self.launch_orientation = ahrs.get_orientation_earth();
                self.ignition_time = Some(Instant::now());
            }
            _ => {
                error!("ignition detected but no AHRS solution available; disabling fins");
                disarm_control();
            }
        }
    }
}

/// Absolute attitude to hold: the orientation captured at ignition, rolled by
/// the commanded angle. This is the guidance half of the control problem, and it
/// lives here because the mission timeline does.
fn attitude_target(launch_orientation: Quat, target_roll: Deg) -> Quat {
    let qroll = roll_deg_to_quat(-target_roll); // Negative for body frame
    // TODO: This probably has different pitch and yaw than what we actually want.
    //  Replace them with the current ones from qcurrent? Or just ignore them
    //  since we're only controlling roll?
    (launch_orientation * qroll).normalized()
}

/// Pre-programmed roll command: returns target roll angle (deg) as a function of time since ignition.
pub fn roll_program(time_since_ignition: Duration) -> Deg {
    let time_s = time_since_ignition.as_micros() as f64 * 1e-6;
    if (0.0..3.0).contains(&time_s) {
        0.0
    } else if (3.0..6.0).contains(&time_s) {
        90.0
    } else {
        0.0
    }
}

pub async fn state_machine_loop() {
    info!("initializing system state");
    let mut system = SystemState::new();
    mark_init_complete(Subsystem::BASE_SYSTEM);
    info!("system state initialized");

    let mut ticker = Ticker::every(Duration::from_hz(100));
    loop {
        ticker.next().await;
        system.handle_state().await;
    }
}