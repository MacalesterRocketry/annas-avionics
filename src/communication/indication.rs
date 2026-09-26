use crate::config::board::{IndicatorsConfig, NUM_LEDS, Neopixel};
use crate::state::{FlightState, GroundSubState};
use crate::utils::errors::{has_any_error, mark_init_complete, report_init_error, Subsystem, SubsystemError};
use crate::{FLIGHT_STATE, Irqs};
use defmt::*;
use defmt_rtt as _;
use embassy_rp::gpio::Output;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Instant, Ticker};
use smart_leds::RGB8;
use smart_leds::hsv::{Hsv, hsv2rgb};

/// Everything that can go wrong initializing the indicators.
///
/// The buzzer and NeoPixel are infallible to construct, so the only failure
/// mode is the flight-state watch running out of receiver slots.
#[derive(Debug, defmt::Format)]
pub enum IndicationError {
    /// Too many `FLIGHT_STATE` receivers are in use, so another can't be created.
    NoFlightStateReceiver,
}

impl SubsystemError for IndicationError {
    fn subsystem(&self) -> Subsystem {
        Subsystem::INDICATORS
    }
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum LedColor {
    Red,
    Orange,
    Yellow,
    Green,
    Cyan,
    Blue,
    Magenta,
    Purple,
    // maybe useful?
    White,
    Off,
}

fn map_color(color: LedColor) -> Hsv {
    match color {
        LedColor::Off => Hsv { hue: 0, sat: 0, val: 0 },
        LedColor::Red => Hsv { hue: 0, sat: 255, val: 255 },
        LedColor::Orange => Hsv { hue: 14, sat: 255, val: 255 },
        LedColor::Yellow => Hsv { hue: 42, sat: 255, val: 255 },
        LedColor::Green => Hsv { hue: 85, sat: 255, val: 255 },
        LedColor::Cyan => Hsv { hue: 127, sat: 255, val: 255 },
        LedColor::Blue => Hsv { hue: 170, sat: 255, val: 255 },
        LedColor::Magenta => Hsv { hue: 212, sat: 255, val: 255 },
        LedColor::Purple => Hsv { hue: 255, sat: 255, val: 255 },
        LedColor::White => Hsv { hue: 0, sat: 0, val: 255 },
    }
}

pub struct StateIndicator {
    pub led: LedColor,
    pub buzzer: BeepCycle<'static>,
}

pub async fn indicator_loop(
    indicators_config: IndicatorsConfig,
) {
    let (mut buzzer, mut neopixel, mut state_receiver) = match init_indicators(indicators_config) {
        Ok(return_val) => {
            mark_init_complete(Subsystem::INDICATORS);
            return_val
        },
        Err(e) => {
            report_init_error(e);
            return;
        }
    };

    // Every 20Hz, check the state and proceed with the according buzzer pattern.
    let mut ticker = Ticker::every(Duration::from_hz(20));
    let mut current_state = state_receiver.try_get().unwrap_or(FlightState::PreLaunch(GroundSubState::Startup));
    let mut entered_at = Instant::now();

    let flash_duration = Duration::from_millis(250);
    let mut last_flash_at = Instant::now(); // for flashing to signal errors
    let mut current_flash_is_orange = false;
    let mut currently_rendered_color: Option<LedColor> = None;
    loop {
        ticker.next().await;
        let state_change = state_receiver.try_changed();
        match state_change {
            Some(new_state) => { // state changed
                info!("State changed: {:?} -> {:?}", current_state, new_state);
                current_state = new_state;
                entered_at = Instant::now();
            }
            None => {}
        }
        let config = current_state.indicator();
        let mut target_color = config.led;

        if has_any_error() {
            if Instant::now() - last_flash_at >= flash_duration {
                current_flash_is_orange = !current_flash_is_orange;
                last_flash_at = Instant::now();
            }
            if current_flash_is_orange {
                target_color = LedColor::Orange;
            }
        } else {
            current_flash_is_orange = false;
        }
        if currently_rendered_color != Some(target_color) { // only update when it actually needs to change color
            set_neopixel_color(&mut neopixel, map_color(target_color), 0.3).await;
            currently_rendered_color = Some(target_color);
        }

        let mut pattern = config.buzzer;
        if has_any_error() {
            if let BeepCycle::Pulse { on_time, .. } = &mut pattern {
                *on_time = Duration::from_secs(1); // make the buzzer beep longer when there's an error
            }
        }
        drive_buzzer(&mut buzzer, pattern, Instant::now() - entered_at).await;
    }
}

fn init_indicators(indicators_config: IndicatorsConfig) -> Result<(Output<'static>, Neopixel, Receiver<'static, CriticalSectionRawMutex, FlightState, 2>), IndicationError> {
    info!("initializing buzzer");
    let mut buzzer = Output::new(indicators_config.buzzer, embassy_rp::gpio::Level::Low);
    buzzer.set_low();
    info!("buzzer initialized");

    info!("initializing NeoPixel");
    let Pio {
        mut common, sm0, ..
    } = Pio::new(indicators_config.neopixel_pio, Irqs);
    let program = PioWs2812Program::new(&mut common);
    let neopixel: Neopixel = PioWs2812::new(
        &mut common,
        sm0,
        indicators_config.neopixel_dma,
        Irqs,
        indicators_config.neopixel,
        &program,
    );
    info!("NeoPixel initialized");

    let receiver = match FLIGHT_STATE.receiver() {
        Some(receiver) => {
            info!("Flight state receiver initialized");
            receiver
        },
        None => return Err(IndicationError::NoFlightStateReceiver),
    };
    Ok((buzzer, neopixel, receiver))
}

pub async fn drive_buzzer(buzzer: &mut Output<'_>, pattern: BeepCycle<'_>, elapsed: Duration) {
    let on = pattern.is_on_at(elapsed);
    buzzer.set_level(on.into());
}

pub async fn set_neopixel_color(
    neopixel: &mut Neopixel,
    color_hsv: Hsv,
    brightness: f32,
) {
    let color_hsv_dimmed = Hsv {
        hue: color_hsv.hue,
        sat: color_hsv.sat,
        val: (brightness * color_hsv.val as f32) as u8,
    };
    let data = [hsv2rgb(color_hsv_dimmed); NUM_LEDS];
    neopixel.write(&data).await;
}

pub async fn set_neopixel_color_rgb(
    neopixel: &mut Neopixel,
    color_rgb: RGB8,
    brightness: f32,
) {
    let color_rgb_dimmed = RGB8 {
        r: (brightness * color_rgb.r as f32) as u8,
        g: (brightness * color_rgb.g as f32) as u8,
        b: (brightness * color_rgb.b as f32) as u8,
    };
    let data = [color_rgb_dimmed; NUM_LEDS];
    neopixel.write(&data).await;
}

pub type BeepSequence<'a> = &'a [(Duration, bool)];

#[derive(Clone, Copy)]
pub enum BeepCycle<'a> {
    Silent,
    /// specified number and duration of beeps
    Pulse {
        count: u32,
        on_time: Duration,
        off_time: Duration,
        cycle_duration: Duration,
    },
    Custom {
        beeps: BeepSequence<'a>,
        cycle_duration: Duration,
    },
}

impl<'a> BeepCycle<'a> {
    pub fn is_on_at(&self, elapsed: Duration) -> bool {
        match self {
            BeepCycle::Silent => false,

            BeepCycle::Pulse { count, on_time, off_time, cycle_duration } => {
                let cycle_ticks = cycle_duration.as_ticks();
                if cycle_ticks == 0 { return false; }

                let time_in_cycle = elapsed.as_ticks() % cycle_ticks;
                let period = on_time.as_ticks() + off_time.as_ticks();
                let sequence_duration = period * (*count as u64);

                if time_in_cycle >= sequence_duration {
                    return false;
                }

                let time_in_pulse = time_in_cycle % period;
                time_in_pulse < on_time.as_ticks() // in one of the beeps?
            }

            BeepCycle::Custom { beeps, cycle_duration } => {
                let total_beeps_duration = beeps.iter().map(|(d, _)| d.as_ticks()).sum();
                let total_cycle_duration = cycle_duration.as_ticks().max(total_beeps_duration);

                if total_cycle_duration == 0 { return false; }
                let mut time_in_cycle = elapsed.as_ticks() % total_cycle_duration;

                for (beep_duration, is_on) in *beeps {
                    let beep_ticks = beep_duration.as_ticks();
                    if time_in_cycle < beep_ticks {
                        return *is_on;
                    }
                    time_in_cycle -= beep_ticks;
                }
                false // In the padding period
            }
        }
    }
}
