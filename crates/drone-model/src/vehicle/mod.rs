use crate::math::atmosphere::AtmosphereModel;
use crate::state::DroneState;
use crate::time::TimeStep;
use nalgebra::{Quaternion, Vector3};

pub mod dynamics_6dof;
pub mod fixed_wing;
pub use fixed_wing::F16Model;
pub mod quadrotor;

pub use dynamics_6dof::{RigidBodyParams, dynamics_6dof};

/// Control input
#[derive(Debug, Clone)]
pub enum KnownActuatorInput {
    Quadrotor(crate::motor::MotorArray<f64>), // [ω0, ω1, ω2, ω3] in rad/s

    FixedWing {
        throttle: f64, // throttle in the range [0, 1]
        aileron: f64,  // aileron - roll in the range [-1, 1]
        elevator: f64, // elevator - pitch in the range [-1, 1]
        rudder: f64,   // rudder - yaw in the range [-1, 1]
    },
}

/// Forces and moments acting on the vehicle
#[derive(Debug, Clone, Default)]
pub struct ForcesAndMoments {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

impl ForcesAndMoments {
    pub fn new(force: Vector3<f64>, torque: Vector3<f64>) -> Self {
        Self { force, torque }
    }

    pub fn add(&self, other: &ForcesAndMoments) -> ForcesAndMoments {
        ForcesAndMoments {
            force: self.force + other.force,
            torque: self.torque + other.torque,
        }
    }
}

/// State derivatives - dynamic functions result
/// dstate/dt at given state and input
#[derive(Debug, Clone)]
pub struct StateDot {
    pub velocity: Vector3<f64>,             // ṗ = v
    pub acceleration: Vector3<f64>,         // v̇ = F/m + g
    pub angular_acceleration: Vector3<f64>, // ω̇ = I⁻¹(τ - ω×Iω)
    pub orientation_dot: Quaternion<f64>,   // q̇ = ½ q ⊗ ω
}

/// AeroModel interface
/// aerodynamic model for the vehicle
pub trait AeroModel: Send + Sync {
    fn compute(
        &self,
        state: &DroneState,
        input: &KnownActuatorInput,
        atmosphere: &dyn AtmosphereModel,
    ) -> ForcesAndMoments;
}

/// VehicleModel interface
pub trait VehicleModel: Send + Sync {
    fn derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot;

    fn step_actuators(&self, state: &mut DroneState, input: &KnownActuatorInput, dt: TimeStep) {
        let _ = (state, input, dt);
    }

    fn equilibrium_input(&self) -> KnownActuatorInput;

    fn gravity(&self) -> f64 {
        9.80665
    }

    fn name(&self) -> &str;

    fn actuator_count(&self) -> usize;

    fn mass(&self) -> f64;
}
