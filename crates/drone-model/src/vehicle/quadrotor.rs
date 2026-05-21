use super::{KnownActuatorInput, StateDot, VehicleModel};
use crate::{
    motor::{Motor, MotorArray},
    state::DroneState,
};
use nalgebra::{Matrix3, Quaternion, Vector3};
use serde::Deserialize;

/// Physical parameters of the drone - constant for given model
/// Loaded from TOML file
/// Control input - angular velocities of all four engines [rad/s]
/// X-frame geometry (top view):
///   1(CCW)  0(CW)
///      \   /
///       [B]     ← nose up (+x)
///      /   \
///   2(CW)  3(CCW)
#[derive(Debug, Clone, Deserialize)]
pub struct QuadrotorParams {
    /// full mass [kg]. Mini 3: 0.249
    pub mass: f64,

    /// arm length: from mass center to engine [m]
    pub arm_length: f64,

    /// thrust coefficient: F = k_thrust * ω²  [N·s²/rad²]
    pub k_thrust: f64,

    /// torque coefficient: τ = k_torque * ω²  [N·m·s²/rad²]
    pub k_torque: f64,

    /// inertia tensor [kg·m²]
    /// for symmetric quadrotor it is diagonal matrix
    /// [[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]]
    #[serde(skip)]
    pub inertia: Matrix3<f64>,

    /// inversion of inertia tensor - computed once, used often
    #[serde(skip)]
    pub inertia_inv: Matrix3<f64>,
}

impl QuadrotorParams {
    /// constructor computing 'inertia_inv'
    pub fn new(
        mass: f64,
        arm_length: f64,
        k_thrust: f64,
        k_torque: f64,
        ixx: f64,
        iyy: f64,
        izz: f64,
    ) -> Self {
        let inertia = Matrix3::from_diagonal(&Vector3::new(ixx, iyy, izz));
        let inertia_inv = inertia
            .try_inverse()
            .expect("Inertia tensor must be invertable");

        Self {
            mass,
            arm_length,
            k_thrust,
            k_torque,
            inertia,
            inertia_inv,
        }
    }

    /// parameters comparable to DJI Mini 3
    /// One engine mass ≈ 0.020 kg
    /// Central mass ≈ 0.249 - 4×0.020 = 0.169 kg
    /// Arm length = 0.085 m
    /// Ixx_engines = 4 × m_engine × r² = 4 × 0.020 × 0.060² = 2.88e-4 kg·m²
    /// Ixx_central ≈ (1/12) × m_central × d² (d ≈ 0.06m body width)
    ///             = (1/12) × 0.169 × 0.06² = 5.1e-5 kg·m²
    /// Ixx ≈ 2.88e-4 + 5.1e-5 ≈ 3.4e-4 kg·m²
    /// Iyy = Ixx
    /// Izz ≈ 3.4e-4 + 3.4e-4 = 6.8e-4 kg·m²
    /// P = 3.71 W (rated), 22.3 W (max)
    /// τ = P / ω
    /// At hover (ω = 632.5 rad/s, P ≈ 3.71 W):
    /// τ_hover = 3.71 / 632.5 ≈ 0.00587 N·m
    /// k_torque = τ / ω² = 0.00587 / 632.5² = 1.466e-8 N·m·s²/rad²
    /// At max RPM (ω = 10700 rpm = 1120 rad/s, P = 22.3 W):
    /// τ_max = 22.3 / 1120 ≈ 0.0199 N·m
    /// k_torque = 0.0199 / 1120² = 1.586e-8
    pub fn mini3() -> Self {
        Self::new(
            0.249,    // mass [kg]
            0.085,    // arm [m] — approximation, Mini 3 has H-frame geometry, not X-frame
            1.526e-6, // k_thrust [N·s²/rad²]
            1.5e-8,   // k_torque [N·m·s²/rad²]
            3.4e-4,   // Ixx [kg·m²]
            3.4e-4,   // Iyy [kg·m²]
            6.8e-4,   // Izz [kg·m²]
        )
    }
}

/// Quadrotor model - implements VehicleModel trait
pub struct QuadrotorModel {
    pub params: QuadrotorParams,
}

impl QuadrotorModel {
    pub fn new(params: QuadrotorParams) -> Self {
        Self { params }
    }

    pub fn mini3() -> Self {
        Self::new(QuadrotorParams::mini3())
    }

    fn motor_thrusts(&self, speeds: &MotorArray<f64>) -> MotorArray<f64> {
        speeds.map(|w| self.params.k_thrust * w * w)
    }

    fn motor_torques(&self, speeds: &MotorArray<f64>) -> MotorArray<f64> {
        speeds.map(|w| self.params.k_torque * w * w)
    }

    fn compute_torques(
        &self,
        thrusts: &MotorArray<f64>,
        torques: &MotorArray<f64>,
    ) -> Vector3<f64> {
        let l = self.params.arm_length;

        // Roll: left engines (1,2) - right (0,3)
        let tau_roll = l
            * ((thrusts[Motor::FrontLeft] + thrusts[Motor::RearLeft])
                - (thrusts[Motor::FrontRight] + thrusts[Motor::RearRight]));

        // Pitch: rear engines (2,3) - front engines (0,1)
        let tau_pitch = l
            * ((thrusts[Motor::RearLeft] + thrusts[Motor::RearRight])
                - (thrusts[Motor::FrontLeft] + thrusts[Motor::FrontRight]));

        // Yaw: CW (0,2) - CCW (1,3)
        let tau_yaw = (torques[Motor::FrontRight] + torques[Motor::RearLeft])
            - (torques[Motor::FrontLeft] + torques[Motor::RearRight]);

        Vector3::new(tau_roll, tau_pitch, tau_yaw)
    }
}

impl VehicleModel for QuadrotorModel {
    /// Main dynamic function
    /// no side effects, no global state
    /// always same result for same arguments
    fn derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot {
        let speeds = match input {
            KnownActuatorInput::Quadrotor(s) => s,
            other => panic!("QuadrotorModel got unexpected input: {:?}", other),
        };

        // Forces and torque from engines
        let p = &self.params;
        let thrusts = self.motor_thrusts(speeds);
        let torques = self.motor_torques(speeds);
        let f_total = thrusts.sum();

        // Translation
        let thrust_body = Vector3::new(0.0, 0.0, f_total);
        let thrust_world = state.orientation * thrust_body;
        let acceleration = thrust_world / p.mass + Vector3::new(0.0, 0.0, -self.gravity());

        // Rotation
        let tau = self.compute_torques(&thrusts, &torques);
        let w = &state.angular_velocity;
        let iw = p.inertia * w;
        let angular_acceleration = p.inertia_inv * (tau - w.cross(&iw));

        // Quaternion
        let omega_quat = Quaternion::from_parts(0.0, *w);
        let orientation_dot = (state.orientation.quaternion() * omega_quat) * 0.5;

        StateDot {
            velocity: state.velocity,
            acceleration,
            angular_acceleration,
            orientation_dot,
        }
    }

    /// hover: all engines at equal speed
    /// analitycally: 4 * k_thrust * ω² = m * g
    /// => ω = sqrt(m * g / (4 * k_thrust))
    fn equilibrium_input(&self) -> KnownActuatorInput {
        let p = &self.params;
        let w = (p.mass * self.gravity() / (4.0 * p.k_thrust)).sqrt();
        KnownActuatorInput::Quadrotor(MotorArray::uniform(w))
    }

    fn name(&self) -> &str {
        "QuadrotorModel (X-frame)"
    }

    fn actuator_count(&self) -> usize {
        4
    }

    fn mass(&self) -> f64 {
        self.params.mass
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::DroneState;
    use nalgebra::{UnitQuaternion, Vector3};

    fn hovering_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        }
    }

    fn hover_input(model: &QuadrotorModel) -> KnownActuatorInput {
        model.equilibrium_input()
    }

    #[test]
    fn hover_gives_zero_acceleration() {
        let model = QuadrotorModel::mini3();
        let dot = model.derivatives(&hovering_state(), &hover_input(&model));
        assert!(
            dot.acceleration.norm() < 1e-6,
            "acc = {:?}",
            dot.acceleration
        );
    }

    #[test]
    fn without_engines_drone_falls() {
        let model = QuadrotorModel::mini3();
        let input = KnownActuatorInput::Quadrotor(MotorArray::uniform(0.0));
        let dot = model.derivatives(&hovering_state(), &input);
        assert!((dot.acceleration.z + 9.81).abs() < 1e-6);
    }

    #[test]
    fn greater_thrust_gives_acceleration_up() {
        let model = QuadrotorModel::mini3();
        let eq = model.equilibrium_input();
        let boosted = match &eq {
            KnownActuatorInput::Quadrotor(s) => KnownActuatorInput::Quadrotor(s.map(|w| w * 1.2)),
            _ => panic!(),
        };
        let dot = model.derivatives(&hovering_state(), &boosted);
        assert!(dot.acceleration.z > 0.0);
    }

    #[test]
    fn roll_right_gives_positive_moment() {
        let model = QuadrotorModel::mini3();
        let eq = model.equilibrium_input();
        let delta = 50.0;
        let input = match &eq {
            KnownActuatorInput::Quadrotor(s) => {
                let mut speeds = *s;
                speeds[Motor::FrontLeft] += delta;
                speeds[Motor::RearLeft] += delta;
                speeds[Motor::FrontRight] -= delta;
                speeds[Motor::RearRight] -= delta;
                KnownActuatorInput::Quadrotor(speeds)
            }
            _ => panic!(),
        };
        let dot = model.derivatives(&hovering_state(), &input);
        assert!(
            dot.angular_acceleration.x > 0.0,
            "angular_acc.x = {}",
            dot.angular_acceleration.x
        );
    }
}
