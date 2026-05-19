use crate::integrator::Integrator;
use drone_model::{dynamics::ControlInput, params::DroneParams, state::DroneState, time::TimeStep};

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
    params: &DroneParams,
    config: &SimConfig,
    integrator: &dyn Integrator,
    mut controller: impl FnMut(&DroneState, f64) -> ControlInput,
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
        state = integrator.step(&state, &input, params, config.dt);
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
        dynamics::ControlInput, motor::MotorArray, params::DroneParams, state::DroneState,
        time::TimeStep,
    };
    use nalgebra::{UnitQuaternion, Vector3};

    fn starting_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        }
    }

    #[test]
    fn hover_keeps_altitude() {
        let params = DroneParams::mini3();
        let config = SimConfig {
            dt: TimeStep::constant(0.005),
            duration: 2.0,
        };
        let integrator = RK4;

        let frames = run(starting_state(), &params, &config, &integrator, |_, _| {
            ControlInput::hover(&DroneParams::mini3())
        });

        let last = &frames.last().unwrap().state;

        // Przy hover wejściu dron powinien zostać blisko z=0
        assert!(
            last.position.z.abs() < 0.01,
            "Dron get away from z=0: z = {:.4}",
            last.position.z
        );
    }

    #[test]
    fn without_engines_drone_falls() {
        let params = DroneParams::mini3();
        let config = SimConfig {
            dt: TimeStep::constant(0.005),
            duration: 1.0,
        };
        let integrator = RK4;

        let frames = run(starting_state(), &params, &config, &integrator, |_, _| {
            ControlInput {
                motor_speeds: MotorArray::uniform(0.0),
            }
        });

        let last = &frames.last().unwrap().state;

        // Po 1 sekundzie dron powinien spaść ~4.9m (½gt²)
        let expected_z = -0.5 * 9.81 * 1.0_f64.powi(2);
        assert!(
            (last.position.z - expected_z).abs() < 0.1,
            "Expected z ≈ {:.2}, computed {:.2}",
            expected_z,
            last.position.z
        );
    }

    #[test]
    fn rk4_more_accurate_than_euler_with_big_dt() {
        use crate::integrator::Euler;

        let params = DroneParams::mini3();

        // Referencja: bardzo mały dt, RK4 — to jest "prawda"
        let config_ref = SimConfig {
            dt: TimeStep::constant(0.0001),
            duration: 1.0,
        };
        let frames_ref = run(starting_state(), &params, &config_ref, &RK4, |_, _| {
            let hover = ControlInput::hover(&DroneParams::mini3());
            ControlInput {
                motor_speeds: hover.motor_speeds.map(|w| w * 1.2),
            }
        });
        let z_ref = frames_ref.last().unwrap().state.position.z;

        // Euler z dużym dt
        let config_big = SimConfig {
            dt: TimeStep::constant(0.05),
            duration: 1.0,
        };
        let frames_euler = run(starting_state(), &params, &config_big, &Euler, |_, _| {
            let hover = ControlInput::hover(&DroneParams::mini3());
            ControlInput {
                motor_speeds: hover.motor_speeds.map(|w| w * 1.2),
            }
        });
        let z_euler = frames_euler.last().unwrap().state.position.z;

        // RK4 z dużym dt
        let frames_rk4 = run(starting_state(), &params, &config_big, &RK4, |_, _| {
            let hover = ControlInput::hover(&DroneParams::mini3());
            ControlInput {
                motor_speeds: hover.motor_speeds.map(|w| w * 1.2),
            }
        });
        let z_rk4 = frames_rk4.last().unwrap().state.position.z;

        let err_euler = (z_euler - z_ref).abs();
        let err_rk4 = (z_rk4 - z_ref).abs();

        assert!(
            err_rk4 < err_euler,
            "RK4 (error={:.4}m) should be more accurate than Euler (error={:.4}m) with reference to z={:.4}m",
            err_rk4,
            err_euler,
            z_ref
        );
    }
}
