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

/// Run an open-loop simulation and return the full frame history.
///
/// The `controller` closure receives the current `DroneState` and the
/// simulation time step `dt`.  Both arguments are identical to `Controller::update`,
/// making it trivial to adapt a `Controller` to this interface if needed.
///
/// Using `dt: TimeStep` (rather than absolute time) matches the Controller
/// trait and avoids the implicit assumption that callers need wall-clock time.
pub fn run(
    initial_state: DroneState,
    model: &dyn VehicleModel,
    config: &SimConfig,
    integrator: &dyn Integrator,
    mut controller: impl FnMut(&DroneState, TimeStep) -> KnownActuatorInput,
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
        let input = controller(&state, config.dt);

        model.step_actuators(&mut state, &input, config.dt);

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
            actuator_state: None,
        }
    }

    #[test]
    fn hover_keeps_altitude() {
        let model = QuadrotorModel::mini3();
        let config = SimConfig {
            dt: TimeStep::constant(0.005),
            duration: 2.0,
        };

        let frames = run(ground_state(), &model, &config, &RK4, |_, _dt| {
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

        let frames = run(ground_state(), &model, &config, &RK4, |_, _dt| {
            KnownActuatorInput::Quadrotor(MotorArray::uniform(0.0))
        });

        let z = frames.last().unwrap().state.position.z;
        // The model uses quadratic drag: F = k_drag * v²
        // Terminal velocity: v_t = sqrt(m*g / k_drag) ≈ 4.0 m/s for Mini 3
        // Analytical z(t) for quadratic drag: -(v_t²/g) * ln(cosh(g*t/v_t))
        // → after 1s: z ≈ -2.9 m (less than free-fall -4.9 m due to drag)
        let m = model.params.mass;
        let g = 9.80665_f64;
        let k = model.params.k_drag;
        let v_t = (m * g / k).sqrt();
        let expected = -(v_t * v_t / g) * (g / v_t).cosh().ln();
        assert!(
            (z - expected).abs() < 0.15,
            "Expected z ≈ {:.2} (quadratic drag), got {:.2}",
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

        let z_ref = run(ground_state(), &model, &config_ref, &RK4, |_, _dt| {
            boosted(&model)
        })
        .last()
        .unwrap()
        .state
        .position
        .z;

        let z_euler = run(ground_state(), &model, &config_big, &Euler, |_, _dt| {
            boosted(&model)
        })
        .last()
        .unwrap()
        .state
        .position
        .z;

        let z_rk4 = run(ground_state(), &model, &config_big, &RK4, |_, _dt| {
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
