use embassy_time::{Duration, Instant};
use serde::{Deserialize, Serialize, Serializer};

pub(crate) mod errors;
pub(crate) mod flags;
pub mod math;
mod hardware_macro;
pub(crate) mod panic_handler;

pub const fn duration_to_seconds(dt: Duration) -> f64 {
    dt.as_nanos() as f64 * 1e-9 // convert to seconds
}

/// Helper function to serialize Option<Instant> as Option<u64> representing microseconds
pub fn serialize_instant_opt<S: Serializer>(instant: &Option<Instant>, serializer: S) -> Result<S::Ok, S::Error> {
    let micros = instant.map(|t| t.as_micros());
    micros.serialize(serializer)
}

pub fn serialize_instant<S: Serializer>(instant: &Instant, serializer: S) -> Result<S::Ok, S::Error> {
    let micros = instant.as_micros();
    micros.serialize(serializer)
}

pub fn deserialize_instant_opt<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<Option<Instant>, D::Error> {
    let micros: Option<u64> = Option::deserialize(deserializer)?;
    Ok(micros.map(|us| Instant::from_micros(us)))
}

pub fn deserialize_instant<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<Instant, D::Error> {
    let micros: u64 = u64::deserialize(deserializer)?;
    Ok(Instant::from_micros(micros))
}

pub fn unwrap_infallible<T>(result: Result<T, core::convert::Infallible>) -> T {
    match result {
        Ok(v) => v,
        Err(e) => match e {},
    }
}