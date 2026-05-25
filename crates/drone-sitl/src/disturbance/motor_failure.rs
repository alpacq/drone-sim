use super::Disturbance;
use drone_model::{
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct MotorFailureConfig {
    pub at_s: f64, // [s]
    pub motor_index: usize,
}

pub struct MotorFailure {
    at_s: f64,
    motor_index: usize,
}

impl MotorFailure {
    pub fn from_config(config: MotorFailureConfig) -> Self {
        Self {
            at_s: config.at_s,
            motor_index: config.motor_index,
        }
    }
}

impl Disturbance for MotorFailure {
    fn is_active(&self, time: f64) -> bool {
        time >= self.at_s
    }

    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep) {
        let eq = model.equilibrium_input();
        let hover_speed = match &eq {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => return,
        };

        let k_torque_approx = 1.5e-8_f64;
        let tau_yaw = k_torque_approx * hover_speed * hover_speed;

        let sign = if self.motor_index % 2 == 0 { 1.0 } else { -1.0 };
        state.angular_velocity.z += sign * tau_yaw * dt.seconds();
    }
}
