use crate::state::DroneState;
use nalgebra::{Quaternion, Vector3};

pub mod fixed_wing;
pub mod quadrotor;

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

/// State derivatives - dynamic functions result
/// dstate/dt at given state and input
#[derive(Debug, Clone)]
pub struct StateDot {
    pub velocity: Vector3<f64>,             // ṗ = v
    pub acceleration: Vector3<f64>,         // v̇ = F/m + g
    pub angular_acceleration: Vector3<f64>, // ω̇ = I⁻¹(τ - ω×Iω)
    pub orientation_dot: Quaternion<f64>,   // q̇ = ½ q ⊗ ω
}

/// VehicleModel interface
pub trait VehicleModel: Send + Sync {
    fn derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot;

    fn equilibrium_input(&self) -> KnownActuatorInput;

    fn gravity(&self) -> f64 {
        9.81
    }

    fn name(&self) -> &str;

    fn actuator_count(&self) -> usize;

    fn mass(&self) -> f64;
}
