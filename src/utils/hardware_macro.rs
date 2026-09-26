/// Note: This file is mostly AI-generated.
/// I'm going to heavily review and refactor it once I get better with Rust macros, I just don't
/// think that's super necessary for now because it seems to work and most likely won't cause any
/// runtime issues.
/// TODO: Figure out this code and refactor
#[macro_export] macro_rules! define_hardware {
    (
        $main_struct:ident {
            // Match all grouped subsystems (e.g., I2cConfig, SdConfig)
            $(
                $group_field:ident : $group_struct:ident {
                    $( $sub_field:ident : $sub_pin:ident ),* $(,)?
                }
            ),* $(,)?
        }
        // Servos get their own section because a PWM slice drives exactly two
        // of them, and which one is channel A vs B is fixed by the GPIO number
        // (even = A, odd = B). Naming the `a:`/`b:` slots here is the only
        // place that fact appears — `control::servo` never sees a slice or a
        // channel. Putting a channel-B pin in an `a:` slot is a compile error,
        // because `Pwm::new_output_ab` demands `ChannelAPin`/`ChannelBPin`.
        //
        // Reads as: `<field on the hardware struct>: <peripherals> -> <outputs>`.
        servos $servo_field:ident : $servo_struct:ident -> $servo_outputs:ident {
            $(
                $slice_field:ident : $slice:ident {
                    a: $a_field:ident = $a_pin:ident,
                    b: $b_field:ident = $b_pin:ident $(,)?
                }
            ),* $(,)?
        }
    ) => {
        // 1. Generate all the sub-structs
        $(
            pub struct $group_struct {
                $( pub $sub_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$sub_pin> ),*
            }
        )*

        // 2. Generate the servo peripherals struct: one slice plus its two pins
        //    per declared pair.
        pub struct $servo_struct {
            $(
                pub $slice_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$slice>,
                pub $a_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$a_pin>,
                pub $b_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$b_pin>,
            )*
        }

        // 3. Generate the per-fin trim table. Same field names as the outputs,
        //    so the compiler catches a fin that gains a pin but no trim. The
        //    values themselves live with the other calibration constants in
        //    `config`, not in this wiring declaration — re-trimming on the
        //    bench should not touch pin assignments.
        #[derive(Debug, Clone, Copy)]
        pub struct Trims {
            $(
                pub $a_field: crate::utils::math::Deg,
                pub $b_field: crate::utils::math::Deg,
            )*
        }

        // 3b. Per-fin deflection sign, for a fin whose linkage or mounting
        //     handedness runs opposite the rest. Same one-per-fin shape as
        //     `Trims` and for the same reason: it is a fact about a physical
        //     servo, so it belongs with the wiring, not with the control law.
        #[derive(Debug, Clone, Copy)]
        pub struct Inverts {
            $(
                pub $a_field: bool,
                pub $b_field: bool,
            )*
        }

        // 3c. Per-fin contribution to each control axis — the airframe geometry
        //     the mixer needs. Keyed by name rather than positional, because
        //     iteration order follows slice declaration and differs between
        //     board revisions: v3 runs xplus/xminus/yplus/yminus while v2 runs
        //     yminus/xplus/xminus/yplus. A positional table would silently mix
        //     the wrong way round on one of them.
        #[derive(Debug, Clone, Copy)]
        pub struct FinMix {
            $(
                pub $a_field: crate::utils::math::AngularVec3,
                pub $b_field: crate::utils::math::AngularVec3,
            )*
        }

        impl FinMix {
            /// Same order as [`iter_mut`](Servos::iter_mut), because both come
            /// from one macro expansion — that is what makes zipping the two
            /// safe without either side naming a fin.
            pub fn iter(&self) -> impl Iterator<Item = &crate::utils::math::AngularVec3> {
                [ $( &self.$a_field, &self.$b_field, )* ].into_iter()
            }
        }

        // 4. Generate the live servo outputs. Each field is an independently
        //    controllable fin — writing one never disturbs its slice-mate,
        //    which `Pwm`'s own `SetDutyCycle` impl cannot promise (it writes
        //    both compare registers at once).
        pub struct $servo_outputs {
            $(
                pub $a_field: crate::control::servo::ServoOutput,
                pub $b_field: crate::control::servo::ServoOutput,
            )*
        }

        impl $servo_struct {
            /// Claim every servo pin, configuring one PWM slice per pair and
            /// splitting it into per-fin channels. `config` applies to every
            /// slice — `top` and `divider` are per-slice registers, so all
            /// servos necessarily share a frame rate.
            pub fn into_servos(
                self,
                config: &embassy_rp::pwm::Config,
                trims: Trims,
                inverts: Inverts,
            ) -> $servo_outputs {
                $(
                    // `new_output_ab` populates both channels, so `split()`
                    // can only yield `(Some, Some)` here — but that is a fact
                    // about the call above, not something the type carries.
                    let (Some($a_field), Some($b_field)) = embassy_rp::pwm::Pwm::new_output_ab(
                        self.$slice_field,
                        self.$a_field,
                        self.$b_field,
                        config.clone(),
                    )
                    .split() else {
                        unreachable!()
                    };
                )*
                $servo_outputs {
                    $(
                        $a_field: crate::control::servo::ServoOutput::new(
                            $a_field, trims.$a_field, inverts.$a_field,
                        ),
                        $b_field: crate::control::servo::ServoOutput::new(
                            $b_field, trims.$b_field, inverts.$b_field,
                        ),
                    )*
                }
            }
        }

        impl $servo_outputs {
            /// Number of servos in this struct.
            pub const COUNT: usize = [$( stringify!($a_field), stringify!($b_field), )*].len();

            /// Every servo, in declaration order. For sweeps and for commands that
            /// apply to all servos at once; individual servos are just named fields.
            pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut crate::control::servo::ServoOutput> {
                [ $( &mut self.$a_field, &mut self.$b_field, )* ].into_iter()
            }

            /// Bring every servo to neutral. Used at init and whenever control is
            /// disarmed, so a disarm can never leave a fin deflected.
            pub fn center_all(&mut self) -> Result<(), embassy_rp::pwm::PwmError> {
                for servo in self.iter_mut() {
                    servo.set_angle(0.0)?;
                }
                Ok(())
            }
        }

        // 5. Generate the main hardware struct
        pub struct $main_struct {
            $( pub $group_field: $group_struct, )*
            pub $servo_field: $servo_struct,
        }

        // 5. Generate the partial-move extraction macro
        #[macro_export]
        macro_rules! take_hardware {
            ($p:expr) => {
                crate::config::board::$main_struct {
                    $(
                        $group_field: crate::config::board::$group_struct {
                            $( $sub_field: $p.$sub_pin ),*
                        },
                    )*
                    $servo_field: crate::config::board::$servo_struct {
                        $(
                            $slice_field: $p.$slice,
                            $a_field: $p.$a_pin,
                            $b_field: $p.$b_pin,
                        )*
                    },
                }
            }
        }
    };
}
