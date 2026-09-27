//! Vec3 / Quat math, ported from C++ `utils.h` + `utils.cpp`.
//!
//! Kept `f64` to preserve the existing AHRS behavior bit-for-bit. The RP2350
//! has only a single-precision FPU, so `f64` math is soft-float — same regime
//! as the RP2040 (M0+, no FPU at all), but with the M33 ALU it's noticeably
//! faster. See PORTING_PLAN.md for the f32 conversion option.

#![allow(clippy::many_single_char_names)]

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};
// libm provides no_std math intrinsics. On host (cargo test) we still use libm
// for bit-identical behavior with the firmware build.
use libm::{asin, atan2, cos, sin, sqrt};
use lsm6dsox::accelerometer::vector::F32x3;
use serde::{Deserialize, Serialize};

pub type Rad = f64;
pub type Deg = f64;

#[inline]
pub const fn rad_to_deg(r: Rad) -> Deg {
    r * 180.0 / core::f64::consts::PI
}

#[inline]
pub const fn deg_to_rad(d: Deg) -> Rad {
    d * core::f64::consts::PI / 180.0
}

#[inline]
pub const fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

// ────────────────────────────────── Vec3 ────────────────────────────────────
#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub const fn dot(self, v: Vec3) -> f64 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    #[inline]
    pub const fn cross(self, v: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * v.z - self.z * v.y,
            y: self.z * v.x - self.x * v.z,
            z: self.x * v.y - self.y * v.x,
        }
    }

    /// L2 norm. Named `norm3` in the C++ source.
    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.dot(self))
    }

    #[inline]
    pub fn mag(self) -> f64 {
        self.norm()
    }

    /// Build a pure quaternion `[w, x, y, z]` from this vector.
    #[inline]
    pub const fn to_quat(self, w: f64) -> Quat {
        Quat::new(w, self.x, self.y, self.z)
    }
}

impl From<nalgebra::Vector3<f64>> for Vec3 {
    #[inline]
    fn from(v: nalgebra::Vector3<f64>) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}
impl From<Vec3> for nalgebra::Vector3<f64> {
    #[inline]
    fn from(v: Vec3) -> Self {
        nalgebra::Vector3::new(v.x, v.y, v.z)
    }
}

impl From<[f64; 3]> for Vec3 {
    #[inline]
    fn from(a: [f64; 3]) -> Self {
        Self { x: a[0], y: a[1], z: a[2] }
    }
}
impl From<Vec3> for [f64; 3] {
    #[inline]
    fn from(v: Vec3) -> Self {
        [v.x, v.y, v.z]
    }
}

impl From<[f32; 3]> for Vec3 {
    #[inline]
    fn from(a: [f32; 3]) -> Self {
        Self { x: a[0] as f64, y: a[1] as f64, z: a[2] as f64 }
    }
}
impl From<Vec3> for [f32; 3] {
    #[inline]
    fn from(v: Vec3) -> Self {
        [v.x as f32, v.y as f32, v.z as f32]
    }
}

impl From<(f64, f64, f64)> for Vec3 {
    #[inline]
    fn from(a: (f64, f64, f64)) -> Self {
        Self { x: a.0, y: a.1, z: a.2 }
    }
}
impl From<Vec3> for (f64, f64, f64) {
    #[inline]
    fn from(v: Vec3) -> Self {
        (v.x, v.y, v.z)
    }
}

impl From<(f32, f32, f32)> for Vec3 {
    #[inline]
    fn from(a: (f32, f32, f32)) -> Self {
        Self { x: a.0 as f64, y: a.1 as f64, z: a.2 as f64 }
    }
}
impl From<Vec3> for (f32, f32, f32) {
    #[inline]
    fn from(v: Vec3) -> Self {
        (v.x as f32, v.y as f32, v.z as f32)
    }
}

impl From<(i16, i16, i16)> for Vec3 {
    #[inline]
    fn from(a: (i16, i16, i16)) -> Self {
        Self { x: a.0 as f64, y: a.1 as f64, z: a.2 as f64 }
    }
}
impl From<Vec3> for (i16, i16, i16) {
    #[inline]
    fn from(v: Vec3) -> Self {
        (v.x as i16, v.y as i16, v.z as i16)
    }
}
impl From<F32x3> for Vec3 {
    #[inline]
    fn from(v: F32x3) -> Self {
        Self { x: v.x as f64, y: v.y as f64, z: v.z as f64 }
    }
}
impl From<Vec3> for F32x3 {
    #[inline]
    fn from(v: Vec3) -> Self {
        F32x3 { x: v.x as f32, y: v.y as f32, z: v.z as f32 }
    }
}

// Scalar ops
impl Mul<f64> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl MulAssign<f64> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        self.x *= s;
        self.y *= s;
        self.z *= s;
    }
}
impl Div<f64> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn div(self, s: f64) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}
impl DivAssign<f64> for Vec3 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}
impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, v: Vec3) -> Vec3 {
        Vec3::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}
impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, v: Vec3) {
        self.x += v.x;
        self.y += v.y;
        self.z += v.z;
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, v: Vec3) -> Vec3 {
        Vec3::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}
impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, v: Vec3) {
        self.x -= v.x;
        self.y -= v.y;
        self.z -= v.z;
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct AngularVec3 {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
}

impl AngularVec3 {
    pub const ZERO: AngularVec3 = AngularVec3 { pitch: 0.0, yaw: 0.0, roll: 0.0 };

    #[inline]
    pub const fn new(pitch: f64, yaw: f64, roll: f64) -> Self {
        Self { pitch, yaw, roll }
    }

    #[inline]
    pub const fn dot(self, v: AngularVec3) -> f64 {
        self.pitch * v.pitch + self.yaw * v.yaw + self.roll * v.roll
    }

    #[inline]
    pub const fn cross(self, v: AngularVec3) -> AngularVec3 {
        AngularVec3 {
            pitch: self.yaw * v.roll - self.roll * v.yaw,
            yaw: self.roll * v.pitch - self.pitch * v.roll,
            roll: self.pitch * v.yaw - self.yaw * v.pitch,
        }
    }

    /// L2 norm. Named `norm3` in the C++ source.
    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.dot(self))
    }

    #[inline]
    pub fn mag(self) -> f64 {
        self.norm()
    }

    /// Build a pure quaternion `[w, x, y, z]` from this vector.
    #[inline]
    pub const fn to_quat(self, w: f64) -> Quat {
        Quat::new(w, self.pitch, self.yaw, self.roll)
    }
}

impl From<Vec3> for AngularVec3 {
    #[inline]
    fn from(v: Vec3) -> Self {
        Self { pitch: v.x, yaw: v.y, roll: v.z } // by the right hand rule, so e.g. roll rotates around the z-axis
    }
}

impl Into<Vec3> for AngularVec3 {
    #[inline]
    fn into(self) -> Vec3 {
        Vec3::new(self.pitch, self.yaw, self.roll)
    }
}

impl From<lsm6dsox::AngularRate> for AngularVec3 {
    #[inline]
    fn from(v: lsm6dsox::AngularRate) -> Self {
        Self { pitch: v.x.as_radians_per_second(), yaw: v.y.as_radians_per_second(), roll: v.z.as_radians_per_second() }
    }
}

impl Into<lsm6dsox::AngularRate> for AngularVec3 {
    #[inline]
    fn into(self) -> lsm6dsox::AngularRate {
        lsm6dsox::AngularRate {
            x: measurements::AngularVelocity::from_radians_per_second(self.pitch),
            y: measurements::AngularVelocity::from_radians_per_second(self.yaw),
            z: measurements::AngularVelocity::from_radians_per_second(self.roll),
        }
    }
}

// Scalar ops
impl Mul<f64> for AngularVec3 {
    type Output = AngularVec3;
    #[inline]
    fn mul(self, s: f64) -> AngularVec3 {
        AngularVec3::new(self.pitch * s, self.yaw * s, self.roll * s)
    }
}
impl MulAssign<f64> for AngularVec3 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        self.pitch *= s;
        self.yaw *= s;
        self.roll *= s;
    }
}
impl Div<f64> for AngularVec3 {
    type Output = AngularVec3;
    #[inline]
    fn div(self, s: f64) -> AngularVec3 {
        AngularVec3::new(self.pitch / s, self.yaw / s, self.roll / s)
    }
}
impl DivAssign<f64> for AngularVec3 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.pitch /= s;
        self.yaw /= s;
        self.roll /= s;
    }
}
impl Add for AngularVec3 {
    type Output = AngularVec3;
    #[inline]
    fn add(self, v: AngularVec3) -> AngularVec3 {
        AngularVec3::new(self.pitch + v.pitch, self.yaw + v.yaw, self.roll + v.roll)
    }
}
impl AddAssign for AngularVec3 {
    #[inline]
    fn add_assign(&mut self, v: AngularVec3) {
        self.pitch += v.pitch;
        self.yaw += v.yaw;
        self.roll += v.roll;
    }
}
impl Sub for AngularVec3 {
    type Output = AngularVec3;
    #[inline]
    fn sub(self, v: AngularVec3) -> AngularVec3 {
        AngularVec3::new(self.pitch - v.pitch, self.yaw - v.yaw, self.roll - v.roll)
    }
}
impl SubAssign for AngularVec3 {
    #[inline]
    fn sub_assign(&mut self, v: AngularVec3) {
        self.pitch -= v.pitch;
        self.yaw -= v.yaw;
        self.roll -= v.roll;
    }
}

// ────────────────────────────────── Quat ────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    #[inline]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Hamilton-conjugate. For a unit quaternion this is also the inverse.
    #[inline]
    pub const fn conjugate(self) -> Quat {
        Quat::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Returns a unit-length copy. Falls back to identity on near-zero norm
    /// (matches C++ behavior — keeps AHRS from blowing up on bad input).
    #[inline]
    pub fn normalized(self) -> Quat {
        let n = self.norm();
        if n <= 1e-15 {
            Quat::IDENTITY
        } else {
            Quat::new(self.w / n, self.x / n, self.y / n, self.z / n)
        }
    }
}

// Scalar ops (must come before Mul<Quat> so they don't shadow it)
impl Mul<f64> for Quat {
    type Output = Quat;
    #[inline]
    fn mul(self, s: f64) -> Quat {
        Quat::new(self.w * s, self.x * s, self.y * s, self.z * s)
    }
}
impl Div<f64> for Quat {
    type Output = Quat;
    #[inline]
    fn div(self, s: f64) -> Quat {
        Quat::new(self.w / s, self.x / s, self.y / s, self.z / s)
    }
}
impl DivAssign<f64> for Quat {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.w /= s;
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}
impl Add for Quat {
    type Output = Quat;
    #[inline]
    fn add(self, q: Quat) -> Quat {
        Quat::new(self.w + q.w, self.x + q.x, self.y + q.y, self.z + q.z)
    }
}
impl Sub for Quat {
    type Output = Quat;
    #[inline]
    fn sub(self, q: Quat) -> Quat {
        Quat::new(self.w - q.w, self.x - q.x, self.y - q.y, self.z - q.z)
    }
}

/// Hamilton product `p ⊗ q`. Matches the C++ definition exactly so AHRS
/// behavior is preserved.
impl Mul<Quat> for Quat {
    type Output = Quat;
    #[inline]
    fn mul(self, q: Quat) -> Quat {
        let p = self;
        Quat::new(
            p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
            p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
            p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
            p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w,
        )
    }
}

impl From<nalgebra::Quaternion<f64>> for Quat {
    #[inline]
    fn from(q: nalgebra::Quaternion<f64>) -> Self {
        Self {
            w: q.w,
            x: q.i,
            y: q.j,
            z: q.k,
        }
    }
}

impl From<Quat> for nalgebra::Quaternion<f64> {
    #[inline]
    fn from(q: Quat) -> Self {
        nalgebra::Quaternion::new(q.w, q.x, q.y, q.z)
    }
}

impl From<Grad4> for Quat {
    fn from(g: Grad4) -> Self {
        Quat::new(g.w, g.x, g.y, g.z)
    }
}

// ─────────────────── Madgwick gradient as a 4-tuple (Grad4) ─────────────────
/// Distinct type from `Quat` so we never accidentally feed a gradient into a
/// Hamilton product. Scalar ops are intentionally limited to what the
/// correction step actually needs.
#[derive(Default, Debug, Clone, Copy, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct Grad4 {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Grad4 {
    #[inline]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }
    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

impl Mul<f64> for Grad4 {
    type Output = Grad4;
    #[inline]
    fn mul(self, s: f64) -> Grad4 {
        Grad4::new(self.w * s, self.x * s, self.y * s, self.z * s)
    }
}
impl Add for Grad4 {
    type Output = Grad4;
    #[inline]
    fn add(self, g: Grad4) -> Grad4 {
        Grad4::new(self.w + g.w, self.x + g.x, self.y + g.y, self.z + g.z)
    }
}
impl DivAssign<f64> for Grad4 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.w /= s;
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}

impl From<Quat> for Grad4 {
    #[inline]
    fn from(q: Quat) -> Grad4 {
        Grad4::new(q.w, q.x, q.y, q.z)
    }
}

// ───────────────────── axis-angle and Euler conversions ─────────────────────
/// Create a quaternion out of a rotation around a given axis.
/// For example, an angle of pi/2 radians around the roll axis creates a quaterion
/// that has rolled from the origin by 90°.
#[inline]
pub fn axis_angle_rad_to_quat(axis: AngularVec3, angle: Rad) -> Quat {
    let half = angle * 0.5;
    let s = sin(half);
    (axis * s).to_quat(cos(half))
}

// TODO: These are all based on the wrong-ish sensor orientation. We should fix this all and transform
//  it in the sensor reads instead.
// TODO: Oddly enough, they might have actually been right? Assuming roll is around the Z axis,
//  this was all right, except that roll was negative. Should test.
#[inline]
pub fn calculate_roll_rad(q: Quat) -> Rad {
    atan2(2.0 * (q.w * q.z + q.x * q.y),
           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

#[inline]
pub fn calculate_pitch_rad(q: Quat) -> Rad {
    atan2(2.0 * (q.w * q.x + q.y * q.z),
          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
}

#[inline]
pub fn calculate_yaw_rad(q: Quat) -> Rad {
    asin(2.0 * (q.w * q.y - q.z * q.x))
}

#[inline]
pub fn calculate_roll_deg(q: Quat) -> Deg { rad_to_deg(calculate_roll_rad(q)) }
#[inline]
pub fn calculate_pitch_deg(q: Quat) -> Deg { rad_to_deg(calculate_pitch_rad(q)) }
#[inline]
pub fn calculate_yaw_deg(q: Quat) -> Deg { rad_to_deg(calculate_yaw_rad(q)) }

#[inline]
pub fn pitch_rad_to_quat(yaw: Rad) -> Quat {
    axis_angle_rad_to_quat(AngularVec3 {pitch: 1.0, yaw: 0.0, roll: 0.0}, yaw)
}

#[inline]
pub fn yaw_rad_to_quat(roll: Rad) -> Quat {
    axis_angle_rad_to_quat(AngularVec3 {pitch: 0.0, yaw: 1.0, roll: 0.0}, roll)
}

#[inline]
pub fn roll_rad_to_quat(pitch: Rad) -> Quat {
    axis_angle_rad_to_quat(AngularVec3 {pitch: 0.0, yaw: 0.0, roll: 1.0}, pitch)
}

#[inline]
pub fn pitch_deg_to_quat(pitch: Deg) -> Quat {
    pitch_rad_to_quat(deg_to_rad(pitch))
}

#[inline]
pub fn yaw_deg_to_quat(yaw: Deg) -> Quat {
    yaw_rad_to_quat(deg_to_rad(yaw))
}

#[inline]
pub fn roll_deg_to_quat(roll: Deg) -> Quat {
    roll_rad_to_quat(deg_to_rad(roll))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-12, "expected {a} ≈ {b}");
    }

    #[test]
    fn identity_is_unit() {
        approx_eq(Quat::IDENTITY.norm(), 1.0);
    }

    #[test]
    fn hamilton_identity_left() {
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        let r = Quat::IDENTITY * q;
        approx_eq(r.w, q.w);
        approx_eq(r.x, q.x);
        approx_eq(r.y, q.y);
        approx_eq(r.z, q.z);
    }

    #[test]
    fn hamilton_identity_right() {
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        let r = q * Quat::IDENTITY;
        approx_eq(r.w, q.w);
        approx_eq(r.x, q.x);
        approx_eq(r.y, q.y);
        approx_eq(r.z, q.z);
    }

    #[test]
    fn conjugate_inverts_unit() {
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        let r = q * q.conjugate();
        approx_eq(r.w, 1.0);
        approx_eq(r.x, 0.0);
        approx_eq(r.y, 0.0);
        approx_eq(r.z, 0.0);
    }

    #[test]
    fn cross_orthogonal() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(a.cross(b), Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn axis_angle_90deg_z() {
        // 90° about +Z = [cos45, 0, 0, sin45]
        let q = axis_angle_rad_to_quat(AngularVec3::new(0.0, 0.0, 1.0), core::f64::consts::FRAC_PI_2);
        approx_eq(q.w, libm::cos(core::f64::consts::FRAC_PI_4));
        approx_eq(q.z, libm::sin(core::f64::consts::FRAC_PI_4));
    }
}
