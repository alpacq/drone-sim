use nalgebra::UnitQuaternion;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EulerAngles {
    /// X axis rotation (left/right roll) [rad]
    pub roll: f64,
    /// Y axis rotation (nose up/down) [rad]
    pub pitch: f64,
    /// Z axis rotation (nose left/right) [rad]
    pub yaw: f64,
}

impl EulerAngles {
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self { roll, pitch, yaw }
    }

    pub fn from_degrees(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            roll: roll.to_radians(),
            pitch: pitch.to_radians(),
            yaw: yaw.to_radians(),
        }
    }

    pub fn to_degrees(&self) -> (f64, f64, f64) {
        (
            self.roll.to_degrees(),
            self.pitch.to_degrees(),
            self.yaw.to_degrees(),
        )
    }
}

///   roll  = atan2(2(qw·qx + qy·qz), 1 - 2(qx² + qy²))
///   pitch = asin(2(qw·qy - qz·qx))          ← osobliwość przy ±90°
///   yaw   = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²))
pub fn quat_to_euler(q: &UnitQuaternion<f64>) -> EulerAngles {
    let w = q.w;
    let x = q.i;
    let y = q.j;
    let z = q.k;

    // Roll
    let sin_roll_cos_pitch = 2.0 * (w * x + y * z);
    let cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y);
    let roll = sin_roll_cos_pitch.atan2(cos_roll_cos_pitch);

    // Pitch — clamp avoids NaN at numeric errors
    let sin_pitch = 2.0 * (w * y - z * x);
    let pitch = sin_pitch.clamp(-1.0, 1.0).asin();

    // Yaw
    let sin_yaw_cos_pitch = 2.0 * (w * z + x * y);
    let cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z);
    let yaw = sin_yaw_cos_pitch.atan2(cos_yaw_cos_pitch);

    EulerAngles { roll, pitch, yaw }
}

/// R = Rz(yaw) · Ry(pitch) · Rx(roll)
pub fn euler_to_quat(e: &EulerAngles) -> UnitQuaternion<f64> {
    let (sr, cr) = (e.roll / 2.0).sin_cos();
    let (sp, cp) = (e.pitch / 2.0).sin_cos();
    let (sy, cy) = (e.yaw / 2.0).sin_cos();

    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_angles_eq(a: EulerAngles, b: EulerAngles, tol: f64) {
        assert!(
            (a.roll - b.roll).abs() < tol,
            "roll:  {} ≠ {}",
            a.roll,
            b.roll
        );
        assert!(
            (a.pitch - b.pitch).abs() < tol,
            "pitch: {} ≠ {}",
            a.pitch,
            b.pitch
        );
        assert!((a.yaw - b.yaw).abs() < tol, "yaw:   {} ≠ {}", a.yaw, b.yaw);
    }

    #[test]
    fn identity_gives_angles_zero() {
        let q = UnitQuaternion::identity();
        let e = quat_to_euler(&q);
        assert_angles_eq(e, EulerAngles::new(0.0, 0.0, 0.0), 1e-10);
    }

    #[test]
    fn round_trip_random_angles() {
        // Konwersja euler→quat→euler powinna dać ten sam wynik
        let original = EulerAngles::new(0.3, -0.2, 1.1);
        let q = euler_to_quat(&original);
        let recovered = quat_to_euler(&q);
        assert_angles_eq(original, recovered, 1e-10);
    }

    #[test]
    fn yaw_90_degrees() {
        let angles = EulerAngles::from_degrees(0.0, 0.0, 90.0);
        let q = euler_to_quat(&angles);
        let recovered = quat_to_euler(&q);
        assert_angles_eq(recovered, EulerAngles::from_degrees(0.0, 0.0, 90.0), 1e-10);
    }

    #[test]
    fn roll_45_degrees() {
        let angles = EulerAngles::from_degrees(45.0, 0.0, 0.0);
        let q = euler_to_quat(&angles);
        let recovered = quat_to_euler(&q);
        assert_angles_eq(recovered, EulerAngles::from_degrees(45.0, 0.0, 0.0), 1e-10);
    }
}
