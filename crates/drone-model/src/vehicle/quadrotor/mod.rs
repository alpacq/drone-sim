use crate::math::atmosphere::{AtmosphereModel, ConstantDensity, Isa};
use crate::motor::{Motor, MotorArray};
use crate::state::{ActuatorState, DroneState};
use crate::time::TimeStep;
use crate::vehicle::dynamics_6dof::{RigidBodyParams, dynamics_6dof};
use crate::vehicle::{AeroModel, ForcesAndMoments, KnownActuatorInput, StateDot, VehicleModel};
use nalgebra::Vector3;

pub mod rotor;
pub use rotor::{QuadrotorRotors, RotorParams, body_drag};

/// Physical parameters of the drone - constant for given model
/// Loaded from TOML file
/// Control input - angular velocities of all four engines [rad/s]
/// X-frame geometry (top view):
///   1(CCW)  0(CW)
///      \   /
///       [B]     ← nose up (+x)
///      /   \
///   2(CW)  3(CCW)
#[derive(Debug, Clone)]
pub struct QuadrotorParams {
    /// full mass [kg]. Mini 3: 0.249
    pub mass: f64,

    /// arm length: from mass center to engine [m]
    pub arm_length: f64,

    /// thrust coefficient: F = k_thrust * ω²  [N·s²/rad²]
    pub k_thrust: f64,

    /// torque coefficient: τ = k_torque * ω²  [N·m·s²/rad²]
    pub k_torque: f64,

    /// body drag coefficient: F_drag = k_drag * v²  [kg/m]
    /// Direction: opposes velocity vector (isotropic quadratic drag)
    /// Terminal velocity: v_t = sqrt(m * g / k_drag)
    /// Mini 3 with k_drag=0.15: v_t ≈ 4.0 m/s (conservative estimate)
    pub k_drag: f64,

    /// rigid body parameters
    pub rigid_body: RigidBodyParams,
}

impl QuadrotorParams {
    /// constructor computing 'inertia_inv'
    pub fn new(
        mass: f64,
        arm_length: f64,
        k_thrust: f64,
        k_torque: f64,
        k_drag: f64,
        ixx: f64,
        iyy: f64,
        izz: f64,
    ) -> Self {
        Self {
            mass,
            arm_length,
            k_thrust,
            k_torque,
            k_drag,
            rigid_body: RigidBodyParams::symmetric(mass, ixx, iyy, izz),
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
            0.15,     // k_drag [kg/m]
            3.4e-4,   // Ixx [kg·m²]
            3.4e-4,   // Iyy [kg·m²]
            6.8e-4,   // Izz [kg·m²]
        )
    }
}

pub struct QuadrotorAero {
    pub params: QuadrotorParams,
}

impl QuadrotorAero {
    fn motor_thrusts(&self, speeds: &MotorArray<f64>) -> MotorArray<f64> {
        speeds.map(|w| self.params.k_thrust * w * w)
    }

    fn motor_torques(&self, speeds: &MotorArray<f64>) -> MotorArray<f64> {
        speeds.map(|w| self.params.k_torque * w * w)
    }
}

impl AeroModel for QuadrotorAero {
    fn compute(
        &self,
        state: &DroneState,
        input: &KnownActuatorInput,
        _atmosphere: &dyn AtmosphereModel,
    ) -> ForcesAndMoments {
        let speeds = match &state.actuator_state {
            Some(ActuatorState::QuadrotorMotors(s)) => s.clone(),
            _ => match input {
                KnownActuatorInput::Quadrotor(s) => s.clone(),
                _ => panic!("QuadrotorAero: unexpected input"),
            },
        };

        let thrusts = self.motor_thrusts(&speeds);
        let torques = self.motor_torques(&speeds);
        let f_total = thrusts.sum();

        let thrust_body = Vector3::new(0.0, 0.0, f_total);

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

        let drag_world = body_drag(&state.velocity, self.params.k_drag);
        let drag_body = state.orientation.inverse() * drag_world;

        ForcesAndMoments {
            force: thrust_body + drag_body,
            torque: Vector3::new(tau_roll, tau_pitch, tau_yaw),
        }
    }
}

/// Quadrotor model - implements VehicleModel trait
pub struct QuadrotorModel {
    pub params: QuadrotorParams,
    pub rotors: QuadrotorRotors,
    pub aero: QuadrotorAero,
    pub atmosphere: Box<dyn AtmosphereModel>,
}

impl QuadrotorModel {
    pub fn new(
        params: QuadrotorParams,
        rotors: QuadrotorRotors,
        atmosphere: Box<dyn AtmosphereModel>,
    ) -> Self {
        let aero = QuadrotorAero {
            params: params.clone(),
        };
        Self {
            params,
            rotors,
            aero,
            atmosphere,
        }
    }

    pub fn mini3() -> Self {
        let params = QuadrotorParams::mini3();
        let hover_speed = (params.mass * 9.80665 / (4.0 * params.k_thrust)).sqrt();
        let rotors = QuadrotorRotors::at_hover(RotorParams::mini3(), hover_speed);
        Self::new(params, rotors, Box::new(Isa))
    }

    pub fn mini3_simple() -> Self {
        let params = QuadrotorParams::mini3();
        let rotors = QuadrotorRotors::new(RotorParams::mini3());
        Self::new(params, rotors, Box::new(ConstantDensity::sea_level()))
    }
}

impl VehicleModel for QuadrotorModel {
    /// Main dynamic function
    /// no side effects, no global state
    /// always same result for same arguments
    fn derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot {
        let mut fm = self.aero.compute(state, input, self.atmosphere.as_ref());

        let gyro = self.rotors.gyroscopic_torque(&state.angular_velocity);

        fm.torque += gyro;

        dynamics_6dof(state, &fm, &self.params.rigid_body, self.gravity())
    }

    fn step_actuators(&self, state: &mut DroneState, input: &KnownActuatorInput, dt: TimeStep) {
        let commanded = match input {
            KnownActuatorInput::Quadrotor(speeds) => speeds,
            _ => return,
        };

        let current = match &state.actuator_state {
            Some(ActuatorState::QuadrotorMotors(s)) => s.clone(),
            _ => commanded.clone(),
        };

        let alpha = (-dt.seconds() / self.rotors.params.time_constant_s).exp();
        let one_minus_alpha = 1.0 - alpha;

        let new_speeds = current.map_with_motor(|m, w_cur| {
            let w_cmd =
                commanded[m].clamp(self.rotors.params.min_speed, self.rotors.params.max_speed);
            w_cur * alpha + w_cmd * one_minus_alpha
        });

        state.actuator_state = Some(ActuatorState::QuadrotorMotors(new_speeds));
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

    fn ground_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    #[test]
    fn hover_gives_zero_acceleration() {
        let model = QuadrotorModel::mini3();
        let dot = model.derivatives(&ground_state(), &(model.equilibrium_input()));
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
        let dot = model.derivatives(&ground_state(), &input);
        assert!((dot.acceleration.z + 9.80665).abs() < 1e-6);
    }

    #[test]
    fn greater_thrust_gives_acceleration_up() {
        let model = QuadrotorModel::mini3();
        let eq = model.equilibrium_input();
        let boosted = match &eq {
            KnownActuatorInput::Quadrotor(s) => KnownActuatorInput::Quadrotor(s.map(|w| w * 1.2)),
            _ => panic!(),
        };
        let dot = model.derivatives(&ground_state(), &boosted);
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
        let dot = model.derivatives(&ground_state(), &input);
        assert!(
            dot.angular_acceleration.x > 0.0,
            "angular_acc.x = {}",
            dot.angular_acceleration.x
        );
    }

    #[test]
    fn hover_gives_zero_acceleration_simple() {
        let model = QuadrotorModel::mini3_simple();
        let input = model.equilibrium_input();
        let dot = model.derivatives(&ground_state(), &input);
        assert!(
            dot.acceleration.norm() < 1e-4,
            "acc = {:?}",
            dot.acceleration
        );
    }

    #[test]
    fn without_engines_drone_falls_simple() {
        let model = QuadrotorModel::mini3_simple();
        let input = KnownActuatorInput::Quadrotor(MotorArray::uniform(0.0));
        let dot = model.derivatives(&ground_state(), &input);
        assert!((dot.acceleration.z + 9.80665).abs() < 0.01);
    }

    #[test]
    fn drag_brakes_at_high_speed_simple() {
        let model = QuadrotorModel::mini3_simple();
        let input = model.equilibrium_input();

        // State with high speed along x-axis
        let fast_state = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::new(20.0, 0.0, 0.0),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };

        let dot = model.derivatives(&fast_state, &input);
        // At 20 m/s, drag should give negative acceleration along x-axis
        assert!(
            dot.acceleration.x < 0.0,
            "Drag should brake: ax = {}",
            dot.acceleration.x
        );
    }
}
