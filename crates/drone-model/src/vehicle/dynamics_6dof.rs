use super::{ForcesAndMoments, StateDot};
use crate::state::DroneState;
use nalgebra::{Matrix3, Quaternion, Vector3};

#[derive(Debug, Clone)]
pub struct RigidBodyParams {
    pub mass: f64,
    pub inertia: Matrix3<f64>,
    pub inertia_inv: Matrix3<f64>,
}

impl RigidBodyParams {
    pub fn new(mass: f64, ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self {
        let inertia = Matrix3::new(ixx, -ixy, -ixz, -ixy, iyy, -iyz, -ixz, -iyz, izz);
        let inertia_inv = inertia
            .try_inverse()
            .expect("Inertia tensor must be invertible");
        Self {
            mass,
            inertia,
            inertia_inv,
        }
    }

    /// Symmetric vehicle (Ixy = Ixz = Iyz = 0).
    /// Used for quadrotor.
    pub fn symmetric(mass: f64, ixx: f64, iyy: f64, izz: f64) -> Self {
        Self::new(mass, ixx, iyy, izz, 0.0, 0.0, 0.0)
    }
}

pub fn dynamics_6dof(
    state: &DroneState,
    fm: &ForcesAndMoments,
    params: &RigidBodyParams,
    gravity: f64,
) -> StateDot {
    // ── Translation ────────────────────────────────────────────────
    //
    // Forces are in body frame — rotate to world frame via quaternion.
    // Add gravity in world frame (always down in ENU convention).
    //
    // F_world = R(q) · F_body
    // a = F_world/m + g_world
    let force_world = state.orientation * fm.force;
    let gravity_world = Vector3::new(0.0, 0.0, -gravity);
    let acceleration = force_world / params.mass + gravity_world;

    // ── Rotation ───────────────────────────────────────────────────
    //
    // Euler's equation: I·ω̇ = τ - ω×(I·ω)
    //
    // Part ω×(I·ω) is gyroscopic effect of rigid body —
    // rotation changes direction of angular momentum → torque.
    // Without this term, the simulator would be unstable at high rotations.
    let w = &state.angular_velocity;
    let iw = params.inertia * w;
    let gyroscopic = w.cross(&iw);
    let angular_acceleration = params.inertia_inv * (fm.torque - gyroscopic);

    // ── Quaternion ────────────────────────────────────────────────
    //
    // q̇ = ½·q⊗ω  (derivative of quaternion from angular velocity)
    //
    // Quaternion must be renormalized after integration —
    // algebraic operation does not preserve |q| = 1.
    let omega_quat = Quaternion::from_parts(0.0, *w);
    let orientation_dot = (state.orientation.quaternion() * omega_quat) * 0.5;

    StateDot {
        velocity: state.velocity,
        acceleration,
        angular_acceleration,
        orientation_dot,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::DroneState;
    use crate::vehicle::ForcesAndMoments;
    use nalgebra::{UnitQuaternion, Vector3};

    fn ground_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    fn mini3_params() -> RigidBodyParams {
        RigidBodyParams::symmetric(0.249, 3.4e-4, 3.4e-4, 6.8e-4)
    }

    #[test]
    fn only_gravity_does_fall() {
        let state = ground_state();
        let fm = ForcesAndMoments::default(); // zero forces
        let params = mini3_params();
        let dot = dynamics_6dof(&state, &fm, &params, 9.80665);

        // Without any forces, only gravity → az = -9.80665
        assert!(
            (dot.acceleration.z + 9.80665).abs() < 1e-6,
            "az = {}",
            dot.acceleration.z
        );
        assert!(dot.acceleration.x.abs() < 1e-10);
        assert!(dot.acceleration.y.abs() < 1e-10);
    }

    #[test]
    fn force_in_vertical_direction_compensates_gravity() {
        let state = ground_state();
        let mass = 0.249_f64;
        let params = mini3_params();

        // Force equal to mg in the vertical direction (in body frame = world frame at identity q)
        let fm = ForcesAndMoments::new(Vector3::new(0.0, 0.0, mass * 9.80665), Vector3::zeros());
        let dot = dynamics_6dof(&state, &fm, &params, 9.80665);

        // Acceleration should be zero
        assert!(
            dot.acceleration.norm() < 1e-6,
            "acc = {:?}",
            dot.acceleration
        );
    }

    #[test]
    fn moment_causes_angular_acceleration() {
        let state = ground_state();
        let params = mini3_params();

        // Moment around the Z axis
        let fm = ForcesAndMoments::new(
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, 1.0), // 1 N·m yaw
        );
        let dot = dynamics_6dof(&state, &fm, &params, 9.80665);

        // α_z = τ_z / Izz
        let expected = 1.0 / 6.8e-4;
        assert!(
            (dot.angular_acceleration.z - expected).abs() < 1.0,
            "α_z = {}",
            dot.angular_acceleration.z
        );
    }

    #[test]
    fn tilted_drone_force_has_horizontal_components() {
        // Drone tilted 45° around the Y axis (pitch)
        let state = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::from_euler_angles(0.0, 45_f64.to_radians(), 0.0),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };
        let mass = 1.0_f64;
        let params = RigidBodyParams::symmetric(mass, 0.1, 0.1, 0.1);

        // Force in the vertical direction in body frame (z_body)
        let fm = ForcesAndMoments::new(Vector3::new(0.0, 0.0, mass * 9.80665), Vector3::zeros());
        let dot = dynamics_6dof(&state, &fm, &params, 9.80665);

        // At 45° pitch: horizontal component of force should be non-zero
        assert!(
            dot.acceleration.x.abs() > 0.1,
            "At 45° pitch: horizontal component of force should be non-zero"
        );
    }
}
