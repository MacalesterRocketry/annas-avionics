use defmt::{bitflags, info, warn};

use crate::utils::errors::{Subsystem, SubsystemError, clear_runtime_error, report_runtime_error};
use crate::utils::flags::{FlagSet, impl_flags};

bitflags! {
    /// One bit per physical chip, indicating that the chip is currently faulted.
    pub struct SensorFault: u8 {
        const LSM  = 1 << 0;
        const LIS3 = 1 << 1;
        const ADXL = 1 << 2;
        const BMP  = 1 << 3;
    }
}
impl_flags!(SensorFault);

static FAULTS: FlagSet<SensorFault> = FlagSet::new();

/// Consecutive failed reads before a chip is declared faulted
const FAULT_THRESHOLD: u8 = 5;
/// Consecutive good reads before a faulted chip is trusted again
const RECOVERY_THRESHOLD: u8 = 5;

/// Chips currently faulted
pub fn faults() -> SensorFault {
    FAULTS.get()
}

/// Mark a sensor as faulted without debouncing, such as if it fails to init
pub fn mark_faulted(sensor: SensorFault) {
    FAULTS.set(sensor);
}

/// Ensures that a single failed read doesn't count as a fault
pub struct FaultDebouncer {
    sensor: SensorFault,
    /// Consecutive outcomes disagreeing with the current faulted state
    consecutive: u8,
    faulted: bool,
}

impl FaultDebouncer {
    pub const fn new(sensor: SensorFault) -> Self {
        Self {
            sensor,
            consecutive: 0,
            faulted: false,
        }
    }

    /// Handles a single read outcome, passing the reading through on success.
    pub fn record<T, E: SubsystemError>(&mut self, outcome: Result<T, E>) -> Option<T> {
        match outcome {
            Ok(reading) => {
                self.record_success();
                Some(reading)
            }
            Err(err) => {
                self.record_failure(err);
                None
            }
        }
    }

    fn record_success(&mut self) {
        if !self.faulted {
            self.consecutive = 0;
            return;
        }
        self.consecutive += 1;
        if self.consecutive >= RECOVERY_THRESHOLD {
            self.faulted = false;
            self.consecutive = 0;
            info!("sensor {} reading again", self.sensor);
            FAULTS.clear(self.sensor);
            if FAULTS.is_empty() {
                clear_runtime_error(Subsystem::SENSORS);
            }
        }
    }

    fn record_failure<E: SubsystemError>(&mut self, err: E) {
        if self.faulted {
            self.consecutive = 0;
            return;
        }
        self.consecutive += 1;
        if self.consecutive >= FAULT_THRESHOLD {
            self.faulted = true;
            self.consecutive = 0;
            FAULTS.set(self.sensor);
            report_runtime_error(err);
        } else {
            // Still a glitch at this point, but a silent one would hide a
            // sensor that is failing intermittently.
            warn!("sensor {} read failed: {}", self.sensor, err);
        }
    }
}
