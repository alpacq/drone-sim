use crate::integrator::Integrator;
use drone_model::{
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};

// One registered step of simulation
#[derive(Debug, Clone)]
pub struct SimFrame {
    pub time: f64,
    pub state: DroneState,
}

// Simulation runtime configuration
pub struct SimConfig {
    pub dt: TimeStep,
    pub duration: f64,
}

// runs simulation and returns history of states
pub fn run(
    initial_state: DroneState,
    model: &dyn VehicleModel,
    config: &SimConfig,
    integrator: &dyn Integrator,
    mut controller: impl FnMut(&DroneState, f64) -> KnownActuatorInput,
) -> Vec<SimFrame> {
    let steps = (config.duration / config.dt.seconds()).ceil() as usize;
    let mut frames = Vec::with_capacity(steps + 1);
    let mut state = initial_state;
    let mut time = 0.0_f64;

    frames.push(SimFrame {
        time,
        state: state.clone(),
    });

    for _ in 0..steps {
        let input = controller(&state, time);
        state = integrator.step(model, &state, &input, config.dt);
        time += config.dt.seconds();
        frames.push(SimFrame {
            time,
            state: state.clone(),
        });
    }

    frames
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::RK4;
    use drone_model::{
        motor::MotorArray,
        state::DroneState,
        time::TimeStep,
        vehicle::{KnownActuatorInput, quadrotor::QuadrotorModel},
    };
    use nalgebra::{UnitQuaternion, Vector3};

    fn ground_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        }
    }

    #[test]
    fn hover_keeps_altitude() {
        let model = QuadrotorModel::mini3();
        let config = SimConfig {
            dt: TimeStep::constant(0.005),
            duration: 2.0,
        };

        let frames = run(ground_state(), &model, &config, &RK4, |_, _| {
            model.equilibrium_input()
        });

        let z = frames.last().unwrap().state.position.z;
        assert!(z.abs() < 0.01, "hover z = {:.4}", z);
    }

    #[test]
    fn without_engines_drone_falls() {
        let model = QuadrotorModel::mini3();
        let config = SimConfig {
            dt: TimeStep::constant(0.005),
            duration: 1.0,
        };

        let frames = run(ground_state(), &model, &config, &RK4, |_, _| {
            KnownActuatorInput::Quadrotor(MotorArray::uniform(0.0))
        });

        let z = frames.last().unwrap().state.position.z;
        let expected = -0.5 * 9.80665 * 1.0_f64.powi(2);
        assert!(
            (z - expected).abs() < 0.1,
            "Expected z ≈ {:.2}, got {:.2}",
            expected,
            z
        );
    }

    #[test]
    fn rk4_more_accurate_than_euler() {
        use crate::integrator::Euler;

        let model = QuadrotorModel::mini3();
        let config_ref = SimConfig {
            dt: TimeStep::constant(0.0001),
            duration: 1.0,
        };
        let config_big = SimConfig {
            dt: TimeStep::constant(0.05),
            duration: 1.0,
        };

        let boosted = |m: &QuadrotorModel| match m.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => KnownActuatorInput::Quadrotor(s.map(|w| w * 1.2)),
            other => other,
        };

        let z_ref = run(ground_state(), &model, &config_ref, &RK4, |_, _| {
            boosted(&model)
        })
        .last()
        .unwrap()
        .state
        .position
        .z;

        let z_euler = run(ground_state(), &model, &config_big, &Euler, |_, _| {
            boosted(&model)
        })
        .last()
        .unwrap()
        .state
        .position
        .z;

        let z_rk4 = run(ground_state(), &model, &config_big, &RK4, |_, _| {
            boosted(&model)
        })
        .last()
        .unwrap()
        .state
        .position
        .z;

        assert!(
            (z_rk4 - z_ref).abs() < (z_euler - z_ref).abs(),
            "RK4 err={:.4}m, Euler err={:.4}m",
            (z_rk4 - z_ref).abs(),
            (z_euler - z_ref).abs()
        );
    }
}
