pub mod aero;
pub mod aero_tables;
pub mod params;
pub mod propulsion;

use crate::math::atmosphere::{AtmosphereModel, Isa};
use crate::state::DroneState;
use crate::time::TimeStep;
use crate::vehicle::{
    KnownActuatorInput, StateDot, VehicleModel,
    dynamics_6dof::{RigidBodyParams, dynamics_6dof},
};
use aero::{AeroState, F16GeomParams, compute_aero};
use propulsion::JetEngine;

pub use params::F16Params;

pub struct F16Model {
    pub params: F16Params,
    pub geom: F16GeomParams,
    pub engine: std::sync::Mutex<JetEngine>,
    pub atmosphere: Box<dyn AtmosphereModel>,
    pub rigid_body: RigidBodyParams,
}

impl F16Model {
    pub fn new(
        params: F16Params,
        geom: F16GeomParams,
        engine: JetEngine,
        atmosphere: Box<dyn AtmosphereModel>,
    ) -> Self {
        let rigid_body = RigidBodyParams::new(
            params.mass,
            params.ixx,
            params.iyy,
            params.izz,
            params.ixy,
            params.ixz,
            params.iyz,
        );
        Self {
            params,
            geom,
            engine: std::sync::Mutex::new(engine),
            atmosphere,
            rigid_body,
        }
    }

    pub fn f16a() -> Self {
        Self::new(
            F16Params::f16a(),
            F16GeomParams::f16a(),
            JetEngine::f110_dry(),
            Box::new(Isa),
        )
    }

    fn current_thrust(&self) -> f64 {
        self.engine.lock().expect("F16 engine mutex").thrust()
    }
}

impl VehicleModel for F16Model {
    fn derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot {
        let aero_state = AeroState::compute(state, self.atmosphere.as_ref());

        let thrust = self.current_thrust();

        let aero_fm = compute_aero(
            &aero_state,
            &state.angular_velocity,
            input,
            &self.geom,
            thrust,
        );

        dynamics_6dof(state, &aero_fm, &self.rigid_body, self.gravity())
    }

    fn step_actuators(&self, state: &mut DroneState, input: &KnownActuatorInput, dt: TimeStep) {
        let throttle = match input {
            KnownActuatorInput::FixedWing { throttle, .. } => *throttle,
            _ => return,
        };

        let altitude = state.position.z.max(0.0);
        let aero = AeroState::compute(state, self.atmosphere.as_ref());

        let mut engine = self.engine.lock().expect("F16 engine mutex");
        engine.step(throttle, altitude, aero.mach, self.atmosphere.as_ref(), dt);

        // Mirror engine state into DroneState so that every DroneState
        // snapshot carries full actuator information (observable, loggable).
        state.actuator_state = Some(
            crate::state::ActuatorState::FixedWingEngine {
                current_throttle: engine.current_throttle,
                current_thrust_n: engine.thrust(),
            },
        );
    }

    fn equilibrium_input(&self) -> KnownActuatorInput {
        KnownActuatorInput::FixedWing {
            throttle: 0.5,
            aileron: 0.0,
            elevator: -0.06,
            rudder: 0.0,
        }
    }

    fn mass(&self) -> f64 {
        self.params.mass
    }

    fn name(&self) -> &str {
        "F-16A (NASA TP-1538)"
    }

    fn actuator_count(&self) -> usize {
        4
    }
}
