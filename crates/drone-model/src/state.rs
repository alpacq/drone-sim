use crate::math::euler::{EulerAngles, quat_to_euler};
use crate::motor::MotorArray;
use nalgebra::{UnitQuaternion, Vector3};

/// Internal state of actuators.
///
/// Stored inside [`DroneState`] so every snapshot is self-contained and
/// simulations can be replayed or logged without additional context.
#[derive(Debug, Clone, PartialEq)]
pub enum ActuatorState {
    /// Four motor speeds [rad/s] for an X-frame quadrotor.
    QuadrotorMotors(MotorArray<f64>),

    /// Jet-engine state for fixed-wing vehicles.
    ///
    /// `current_throttle` is the first-order-filtered throttle command [0, 1].
    /// `current_thrust_n` is the resulting engine thrust in Newtons.
    ///
    /// NOTE: `F16Model` still keeps a `Mutex<JetEngine>` internally for
    /// compatibility.  `step_actuators` writes the same values here so that
    /// callers can *read* the engine state from a `DroneState` without
    /// accessing the model.  Full deterministic replay from state alone is a
    /// future improvement (tracked in the issue log).
    FixedWingEngine {
        current_throttle: f64,
        current_thrust_n: f64,
    },
}

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

    /// optional state of actuators
    pub actuator_state: Option<ActuatorState>,
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
            actuator_state: None,
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
            actuator_state: None,
        };
        let s2 = s.clone();
        assert_eq!(s.position, s2.position);
    }
}
