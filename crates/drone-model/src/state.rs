use crate::math::euler::{EulerAngles, quat_to_euler};
use nalgebra::{UnitQuaternion, Vector3};

/// Full drone state in time t
///
/// All values are in world frame, apart from 'angular_velocity', which is in body frame
/// Units: m, s, rad
#[derive(Debug, Clone)]
pub struct DroneState {
    /// [x, y, z] position in m, z pointing up
    pub position: Vector3<f64>,

    /// linear velocity [vx, vy, vz] in m/s
    pub velocity: Vector3<f64>,

    /// angular velocity [p, q, r] in rad/s, in body frame
    pub angular_velocity: Vector3<f64>,

    /// drone's orientation as unit quaternion
    /// represents rotation between world frame and body frame
    pub orientation: UnitQuaternion<f64>,
}

impl DroneState {
    /// Orientation view as Euler angles ZYX
    /// Only for visualization and comparison with DJI telemetry
    pub fn euler_angles(&self) -> EulerAngles {
        quat_to_euler(&self.orientation)
    }

    pub fn on_ground() -> Self {
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_clones() {
        let s = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        };
        let s2 = s.clone();
        assert_eq!(s.position, s2.position);
    }
}
