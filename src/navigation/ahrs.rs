// TODO: These are all pure functions, so this is all unit testable without even needing to run on the board.
//  Figure out a way to do that.

use crate::utils::{serialize_instant, deserialize_instant};
use crate::config::{AHRS_ACC_BETA, AHRS_MAG_BETA, G, GYRO_LPF_HZ};
use crate::utils::duration_to_seconds;
use crate::utils::math::{AngularVec3, Grad4, Quat, Vec3, axis_angle_rad_to_quat};
use core::f64::consts::PI;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant};
use libm::sqrt;
use serde::{Deserialize, Serialize};

/// Latest AHRS solution, published once per sensor sample.
pub static AHRS_STATE: Watch<CriticalSectionRawMutex, AhrsState, 2> = Watch::new();

/// Mutable AHRS runtime state. Owned by the sensor task, which calls
/// [`update`](AhrsState::update) each sample and publishes the result to
/// [`AHRS_STATE`] for the control loop to read.
// #[derive(Debug, Clone, Copy)]
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct AhrsState {
    pub q: Quat,
    #[serde(serialize_with = "serialize_instant")]
    #[serde(deserialize_with = "deserialize_instant")]
    pub last_update: Instant,
    pub acceleration_earth: Vec3,
    pub velocity_earth: Vec3, // TODO: Might not be a bad idea to make earth-frame and body-frame separate types with into() between them
    pub position_earth: Vec3,
    pub angular_velocity_body: AngularVec3,
    /// [`angular_velocity_body`](Self::angular_velocity_body) low-passed at
    /// [`GYRO_LPF_HZ`]. Use this for control; the raw field is for logging.
    pub angular_velocity_filtered: AngularVec3,
    in_flight: bool,
    // TODO: Add some sort of validity flag here to disable the control system if we don't have good gyro or AHRS hasn't settled?
}

impl Default for AhrsState {
    fn default() -> Self {
        Self {
            q: Quat::IDENTITY,
            last_update: Instant::now(),
            acceleration_earth: Vec3::ZERO,
            velocity_earth: Vec3::ZERO,
            position_earth: Vec3::ZERO,
            angular_velocity_body: AngularVec3::ZERO,
            angular_velocity_filtered: AngularVec3::ZERO,
            in_flight: false,
        }
    }
}

impl AhrsState {
    pub fn update(&mut self, gyro: AngularVec3, accel: Option<Vec3>, mag: Option<Vec3>, now: Instant) {
        let dt = now - self.last_update;
        self.last_update = now;

        // usually like .005s, so if it's far greater, skip so we don't get huge jumps in orientation from bad timing
        if dt <= Duration::from_millis(0) || dt > Duration::from_millis(100) {
            return; // Invalid time step, skip update
        }

        //  COMPLETE Q0 → Q4
        // Step 0: Initial quaternion (q0) - previous orientation
        let q0 = self.q;

        // Step 2: Gyro propagation (q1) - integrate angular rates
        let delta_q = delta_quat_from_gyro(gyro, dt);
        let q1 = q0 * delta_q;

        // Step 3: Normalize (q2) - maintain quaternion unit length
        let q2 = q1.normalized();

        // Step 4: Madgwick sensor fusion correction (q3) - fuse with accelerometer and magnetometer
        // In flight, we just follow the gyroscope. Gravity doesn't affect it, so we need to ignore the accelerometer, and the magnetometer is unreliable.
        let q3 = if self.in_flight { q2 } else { madgwick_correction_step(q2, accel, mag, dt) };

        // Step 5: Final normalization (q4) - ensure valid quaternion
        let q4 = q3.normalized();

        // Update state with final quaternion, final state
        self.q = q4;

        // EARTH FRAME CONVERSIONS
        // Convert body-frame measurements to earth frame for control systems
        if let Some(accel) = accel { // just skip applying acceleration if we don't have it
            let earth_accel = rotate_body_to_earth(q4, accel);
            self.acceleration_earth = earth_accel - Vec3 {x: 0.0, y: 0.0, z: G}; // Remove gravity from vertical acceleration
        }
        let dt_seconds = duration_to_seconds(dt);
        self.velocity_earth += self.acceleration_earth * dt_seconds;
        self.position_earth += self.velocity_earth * dt_seconds;

        self.angular_velocity_body = gyro; // still in body frame, but we can use it for control

        // Anti-alias the gyro for the 50 Hz control loop, which cannot undo
        // this after the fact — see GYRO_LPF_HZ. Alpha is derived from the
        // measured dt rather than hardcoded so loop jitter doesn't drag the
        // cutoff around with it.
        let tau = 1.0 / (2.0 * PI * GYRO_LPF_HZ);
        let alpha = dt_seconds / (dt_seconds + tau);
        self.angular_velocity_filtered += (gyro - self.angular_velocity_filtered) * alpha;

        // // TODO: figure out debug stuff for Rust
        // // CONTINUOUS ORIENTATION MONITORING/Active Tracking
        // #if DEBUG and DEBUG_PRINT_ORIENTATION
        // static unsigned long lastPrint = 0;
        // if (now_micros - lastPrint > 100000) {
        //     // Convert quaternion to Euler angles
        //     const double roll = calculate_roll_deg(self.q);
        //     const double pitch = calculate_pitch_deg(self.q);
        //     const double yaw = calculate_yaw_deg(self.q);
        //
        //     // Display orientation and earth-frame data
        //     Serial.println("ROCKET ORIENTATION");
        //     Serial.printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\r\n",
        //                   self.q.w, self.q.x, self.q.y, self.q.z);
        //     Serial.printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n",
        //                   roll, pitch, yaw);
        //     Serial.printf("Earth Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\r\n",
        //                   self.acceleration_earth.x, self.acceleration_earth.y, self.acceleration_earth.z);
        //     Serial.printf("Rocket Accel: X=%.2f, y=%.2f, Z=%.2f m/s²\r\n",
        //                   accel.x, accel.y, accel.z);
        //     Serial.printf("Earth Velocity: X=%.2f, Y=%.2f, Z=%.2f m/s\r\n",
        //                   self.velocity_earth.x, self.velocity_earth.y, self.velocity_earth.z);
        //     Serial.printf("Earth Position: X=%.2f, Y=%.2f, Z=%.2f m\r\n",
        //                   self.position_earth.x, self.position_earth.y, self.position_earth.z);
        //     Serial.println("==========================");
        //
        //     lastPrint = now_micros;
        // }
        // #endif
    }

    pub const fn launch(&mut self) {
        self.in_flight = true;
    }

    pub const fn landing(&mut self) {
        self.in_flight = false;
    }

    pub const fn zero_pos_vel(&mut self) {
        self.position_earth = Vec3::ZERO;
        self.velocity_earth = Vec3::ZERO;
    }

    // TODO: should this function exist? Should it just be new()?
    pub fn start(&mut self) {
        self.last_update = Instant::now();
    }

    // TODO: Right now, other modules can access the struct directly. Either remove these
    //  or remove public access to the struct. (and probably figure out some way of making
    //  it all thread-safe)
    pub const fn get_orientation_earth(&self) -> Quat { self.q }
    pub const fn get_acceleration_earth(&self) -> Vec3 { self.acceleration_earth }
    pub const fn get_velocity_earth(&self) -> Vec3 { self.velocity_earth }
    pub const fn get_position_earth(&self) -> Vec3 { self.position_earth }
    pub const fn get_angular_velocity_body(&self) -> AngularVec3 { self.angular_velocity_body }
    /// Low-passed body rates. This is the one control should use — see [`GYRO_LPF_HZ`].
    pub const fn get_angular_velocity_filtered(&self) -> AngularVec3 { self.angular_velocity_filtered }
}

// build delta quaternion(propagation) from angular rate omega (rad/s) and dt
pub fn delta_quat_from_gyro(omega: AngularVec3, dt: Duration) -> Quat {
    let dt_s = duration_to_seconds(dt);
    let wmag = omega.norm();
    if wmag < 1e-12 {
        // tiny rotation -> small-angle approx: q ≈ [1, 0.5*ω*dt]
        (omega * 0.5 * dt_s).to_quat(1.0)
    } else {
        let axis = omega / wmag;
        let theta = wmag * dt_s;
        axis_angle_rad_to_quat(axis, theta)
    }
}

// Madgwick correction step, I dont know how to get the library from adafruit so I just did it manually/mathmatically!

pub fn compute_magnetometer_gradient(m_n: Vec3, q: Quat) -> Grad4 {
    // For magnetometer part we should compute reference direction and its gradient.
    // compute Earth's magnetic field in body frame and gradient.
    // Compute h = q ⊗ m_n ⊗ q*  (magnetic field in earth frame)
    let h = rotate_body_to_earth(q, m_n);

    // Projection of h onto x-y plane of Earth (reference)
    // b = [0, bx, 0, bz] as in Madgwick
    let b = Vec3 {
        x: sqrt(h.x * h.x + h.y * h.y),
        y: 0.0,
        z: h.z
    };

    // compute the gradient of magnetometer using a simplified combined gradient:
    // Compute an approximate mag error vector between predicted and measured (in body frame!)
    let m_pred = rotate_earth_to_body(q, b); // predicted magnetic field in body frame (reference)
    let mag_err = Vec3 {
        x: m_pred.x / (m_pred.norm() + 1e-12) - m_n.x,
        y: m_pred.y / (m_pred.norm() + 1e-12) - m_n.y,
        z: m_pred.z / (m_pred.norm() + 1e-12) - m_n.z
    };

    // Build a simple magnetometer gradient approximation using cross products of predicted vs measured
    // This is less exact than full Madgwick derivation but should work well in practice as a correction term, simpler for us
    let g_mag_vec = Vec3 {
        x: 2.0 * (q.y * mag_err.z - q.z * mag_err.y + q.w * mag_err.x - q.x * mag_err.z),
        y: 2.0 * (q.z * mag_err.x - q.w * mag_err.z + q.x * mag_err.y - q.y * mag_err.x),
        z: 2.0 * (q.w * mag_err.y - q.x * mag_err.x + q.y * mag_err.z - q.z * mag_err.y)
    };
    // Convert this vector into 4-component approximate gradient (spread across all axes)
    Grad4 {
        w: 0.0,
        x: g_mag_vec.x,
        y: g_mag_vec.y,
        z: g_mag_vec.z
    }
}

pub fn compute_accelerometer_gradient(a_n: Vec3, q: Quat) -> Grad4 {
    // Compute objective function gradient (following Madgwick 2010 equations, check paper).
    // For full derivation see Madgwick's report; here we implement the standard gradient step.
    // variables
    let _2q1 = 2.0 * q.w; let _2q2 = 2.0 * q.x; let _2q3 = 2.0 * q.y; let _2q4 = 2.0 * q.z;
    let _4q2 = 4.0 * q.x; let _4q3 = 4.0 * q.y;

    // Gradient of f (accel) part (from Madgwick): need to use specific q# combinations
    // math behind the values= f = predicted_gravity - measured_gravity; predicted_gravity = [2(q2 q4 - q1 q3), 2(q1 q2 + q3 q4), q1^2 - q2^2 - q3^2 + q4^2]
    let f = Vec3 {
        x: 2.0 * (q.x * q.z - q.w * q.y) - a_n.x,
        y: 2.0 * (q.w * q.x + q.y * q.z) - a_n.y,
        z: q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z - a_n.z
    };

    // Jacobian J (3x4) lines combined into gradient g = J^T * f  (this expands to 4 components, as we need it to build a quaternion)
    // Madgwick 2010 paper's compact expression:
    Grad4 {
        w: 0.0,
        x: -_2q3 * f.x + _2q2 * f.y,
        y:  _2q4 * f.x + _2q1 * f.y - _4q2 * f.z,
        z: -_2q1 * f.x + _2q4 * f.y - _4q3 * f.z,
    }
}

// returns corrected sensor fused quaternion as quaternion q3 (unnormalized)
fn madgwick_correction_step(q_pred: Quat, // predicted quaternion (gyro-propagated & normalized) -> q2
                            acc: Option<Vec3>, // accelerometer (bias-corrected) in body frame (no normalizing required; we can normalize inside)
                            mag: Option<Vec3>, // magnetometer (bias-corrected & soft-iron corrected) in body frame
                            dt: Duration) -> Quat { // time step (s)
    const NO_CORRECTION: Grad4 = Grad4 {w: 0.0, x: 0.0, y: 0.0, z: 0.0};

    let q = q_pred;

    let mag_grad = match mag {
        Some(mag) => {
            let nm = mag.norm();
            // Skip magnetometer correction if the measurement is too small
            if nm >= 1e-12 {
                let m_n = mag / nm; // normalized magnetometer measurement
                compute_magnetometer_gradient(m_n, q)
            } else {
                NO_CORRECTION
            }
        }
        None => NO_CORRECTION,
    };

    let accel_grad = match acc {
        Some(acc) => {
            let na = acc.norm();
            // Skip gravity-based accelerometer correction if the measurement doesn't seem like gravity (like during burn or coast)
            if na >= 0.5 * G && na <= 2.0 * G {
                let a_n = acc / na; // normalized accelerometer measurement
                compute_accelerometer_gradient(a_n, q)
            } else {
                NO_CORRECTION
            }
        }
        None => NO_CORRECTION,
    };

    let mut g_combined = accel_grad * AHRS_ACC_BETA + mag_grad * AHRS_MAG_BETA;

    // Normalize gradient
    let gn = g_combined.norm();
    if gn > 0.0 {
        g_combined /= gn;
    }

    // Integrate to get corrected quaternion (Simple Euler integration)
    q_pred - (g_combined * duration_to_seconds(dt)).into()
}

// Bias computation (mean across samples)
pub fn compute_bias_mean(samples: &[Vec3]) -> Vec3 {
    let mut s = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    if samples.is_empty() {
         return s;
    }
    for v in samples.iter() {
        s += *v;
    }
    s / samples.len() as f64
}

pub fn rotate_body_to_earth(q: Quat, v_b: Vec3) -> Vec3 {
    // p = q ⊗ [0,v_b] ⊗ q*
    let res = q * v_b.to_quat(0.0) * q.conjugate();
    Vec3{ x: res.x, y: res.y, z: res.z } // last 3 are vector part
}

pub fn rotate_earth_to_body(q: Quat, v_e: Vec3) -> Vec3 {
    // p = q* ⊗ [0,v_e] ⊗ q
    let res = q.conjugate() * v_e.to_quat(0.0) * q;
    Vec3{ x: res.x, y: res.y, z: res.z } // last 3 are vector part
}
