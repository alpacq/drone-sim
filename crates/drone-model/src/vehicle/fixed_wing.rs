use super::{KnownActuatorInput, StateDot, VehicleModel};
use crate::state::DroneState;
use nalgebra::{Quaternion, Vector3};
use serde::Deserialize;

/// Physical parameters of the drone - constant for given model
/// Loaded from TOML file
#[derive(Debug, Clone, Deserialize)]
pub struct FixedWingParams {
    pub mass: f64,        // [kg]
    pub wing_area: f64,   // [m^2]
    pub wingspan: f64,    // [m]
    pub max_thrust: f64,  // [N]
    pub cl0: f64,         // [-] lift coefficient at zero angle of attack
    pub cl_alpha: f64,    // [1/rad] lift coefficient derivative
    pub cd0: f64,         // [-] drag coefficient at zero angle of attack
    pub air_density: f64, // [kg/m^3]
}

impl FixedWingParams {
    /// Small model airplane parameters (~1kg)
    pub fn small_plane() -> Self {
        Self {
            mass: 1.0,
            wing_area: 0.15,
            wingspan: 1.0,
            max_thrust: 15.0,
            cl0: 0.3,
            cl_alpha: 5.7,
            cd0: 0.02,
            air_density: 1.225,
        }
    }
}

pub struct FixedWingModel {
    pub params: FixedWingParams,
}

impl FixedWingModel {
    pub fn new(params: FixedWingParams) -> Self {
        Self { params }
    }
}

impl VehicleModel for FixedWingModel {
    fn derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot {
        let (throttle, _aileron, elevator, _rudder) = match input {
            KnownActuatorInput::FixedWing {
                throttle,
                aileron,
                elevator,
                rudder,
            } => (*throttle, *aileron, *elevator, *rudder),
            other => panic!("FixedWingModel: got unexpected input: {:?}", other),
        };

        let p = &self.params;
        let v = state.velocity;
        let v_norm = v.norm();

        // Aerodynamic force in body frame
        // Angle of attack - simplified 2D model
        let alpha = if v_norm > 0.1 {
            (v.z / v_norm).asin()
        } else {
            0.0
        };

        let q_dyn = 0.5 * p.air_density * v_norm * v_norm;

        // Lift
        let cl = p.cl0 + p.cl_alpha * (alpha + elevator * 0.3);
        let lift = cl * q_dyn * p.wing_area;

        let cd = p.cd0 + cl * cl / (std::f64::consts::PI * 8.0);
        let drag = cd * q_dyn * p.wing_area;

        // Thrust
        let thrust = throttle * p.max_thrust;

        // Forces in body frame
        let force_body = Vector3::new(thrust - drag, 0.0, lift);

        // Translation
        let force_world = state.orientation * force_body;
        let gravity = Vector3::new(0.0, 0.0, -self.gravity() * p.mass);
        let acceleration = (force_world + gravity) / p.mass;

        // Rotation
        // TODO: full aerodynamic moments (Cm, Cl, Cn)
        let angular_acceleration = Vector3::zeros();

        let w = &state.angular_velocity;
        let omega_quat = Quaternion::from_parts(0.0, *w);
        let orientation_dot = (state.orientation.quaternion() * omega_quat) * 0.5;

        StateDot {
            velocity: state.velocity,
            acceleration,
            angular_acceleration,
            orientation_dot,
        }
    }

    fn equilibrium_input(&self) -> KnownActuatorInput {
        // vertical flight: throttle balances drag, elevator = 0
        KnownActuatorInput::FixedWing {
            throttle: 0.3,
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
        }
    }

    fn name(&self) -> &str {
        "Simplified FixedWingModel"
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

    #[test]
    fn compiles_and_returns_stat_dot() {
        let model = FixedWingModel::new(FixedWingParams::small_plane());
        let state = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::new(15.0, 0.0, 0.0),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        };
        let input = model.equilibrium_input();
        let _ = model.derivatives(&state, &input);
    }
}
