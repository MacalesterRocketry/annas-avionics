//! Atomic bitflag sets, used by [`crate::utils::errors`] and [`crate::sensors::health`].
//! Users don't get to actually manipulate the bits, just the flags they represent.

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

/// A flag set with 8 bits based on `defmt::bitflags!`. Don't implement this manually, use the `impl_flags!` macro instead.
pub trait Flags: Copy {
    fn bits(&self) -> u8;
    fn from_bits_truncate(bits: u8) -> Self;
}

/// Implements [`Flags`] for a `u8`-backed `defmt::bitflags!` struct.
macro_rules! impl_flags {
    ($ty:ty) => {
        impl $crate::utils::flags::Flags for $ty {
            fn bits(&self) -> u8 {
                <$ty>::bits(self)
            }
            fn from_bits_truncate(bits: u8) -> Self {
                <$ty>::from_bits_truncate(bits)
            }
        }
    };
}
pub(crate) use impl_flags;

/// A set of atomic `F` flags with a specialized interface, which works across cores.
pub struct FlagSet<F: Flags> {
    bits: AtomicU8,
    _flags: PhantomData<F>,
}

impl<F: Flags> FlagSet<F> {
    pub const fn new() -> Self {
        Self {
            bits: AtomicU8::new(0),
            _flags: PhantomData,
        }
    }

    /// Every flag currently set.
    pub fn get(&self) -> F {
        F::from_bits_truncate(self.load())
    }

    /// Add `flags` to the set (OR), leaving the rest alone.
    pub fn set(&self, flags: F) {
        self.bits.fetch_or(flags.bits(), Ordering::Release);
    }

    /// Remove `flags` from the set (NAND), leaving the rest alone.
    pub fn clear(&self, flags: F) {
        self.bits.fetch_and(!flags.bits(), Ordering::Release);
    }

    /// True if *every* flag in `flags` is set.
    pub fn contains(&self, flags: F) -> bool {
        self.load() & flags.bits() == flags.bits()
    }

    /// True if no flag is set.
    pub fn is_empty(&self) -> bool {
        self.load() == 0
    }

    fn load(&self) -> u8 {
        self.bits.load(Ordering::Acquire)
    }
}
