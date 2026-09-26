use crate::utils::flags::{FlagSet, impl_flags};
use defmt::{Format, bitflags, error, info};

bitflags! {
    pub struct Subsystem: u8 {
        const BASE_SYSTEM = 1 << 0; // core 0
        const SENSORS     = 1 << 1;
        const CONTROL     = 1 << 2;

        const INDICATORS  = 1 << 5; // core 1
        const SD_CARD     = 1 << 6;
        const GPS         = 1 << 7;
    }
}
impl_flags!(Subsystem);

/// Subsystems that finished initializing successfully.
pub static INIT_DONE: FlagSet<Subsystem> = FlagSet::new();
/// Subsystems that tried to initialize and settled into a failed state.
pub static INIT_FAILURES: FlagSet<Subsystem> = FlagSet::new();
pub static RUNTIME_FAILURES: FlagSet<Subsystem> = FlagSet::new();

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
pub fn mark_init_complete(subsystem: Subsystem) {
    info!("subsystem {} initialized", subsystem);
    INIT_DONE.set(subsystem)
}
/// True once every subsystem has reported an init outcome, successful or not.
/// Makes it possible to differentiate an error from a subsystem that hasn't
/// finished initializing, so the avionics can transition from Startup to whatever
/// state makes sense given the presence or absence of any particular errors.
pub fn is_init_settled() -> bool {
    (INIT_DONE.get() | INIT_FAILURES.get()) == Subsystem::all()
}
/// Clear `subsystem`'s runtime-failure bit, announcing that it is working again.
pub fn clear_runtime_error(subsystem: Subsystem) {
    if has_runtime_error(subsystem) {
        info!("subsystem {} working again, clearing runtime error", subsystem);
    }
    // TODO: This should log to the SD card to say there's no longer an error
    RUNTIME_FAILURES.clear(subsystem);
}
pub fn has_runtime_error(subsystem: Subsystem) -> bool {
    RUNTIME_FAILURES.contains(subsystem)
}
pub fn has_any_runtime_error() -> bool {
    !RUNTIME_FAILURES.is_empty()
}
pub fn has_any_init_error() -> bool {
    !INIT_FAILURES.is_empty()
}
pub fn has_any_error() -> bool {
    has_any_init_error() || has_any_runtime_error()
}

// ---------------------------------------------------------------------------
// Subsystem error reporting
// ---------------------------------------------------------------------------

/// An error that can be attributed to exactly one [`Subsystem`].
///
/// Implement this for each subsystem's error enum. Call sites then report
/// failures through [`report_init_error`] / [`report_runtime_error`] using this error type.
pub trait SubsystemError: Format {
    fn subsystem(&self) -> Subsystem;
}

/// Log `err` and mark its subsystem as having failed to initialize.
pub fn report_init_error<E: SubsystemError>(err: E) {
    let subsystem = err.subsystem();
    error!("init error: subsystem {} failed to initialize: {}", subsystem, err);
    INIT_FAILURES.set(subsystem)
}

/// Log `err` and set its subsystem's runtime-failure bit. This latches the bit, so
/// use [`clear_runtime_error`] once the subsystem recovers.
pub fn report_runtime_error<E: SubsystemError>(err: E) {
    // TODO: This should probably send a message to the SD card.
    //  Same for init.
    let subsystem = err.subsystem();
    error!("runtime error: subsystem {} failed at runtime: {}", subsystem, err);
    RUNTIME_FAILURES.set(subsystem)
}