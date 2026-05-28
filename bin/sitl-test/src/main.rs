use anyhow::Result;
use drone_control::cascade::make_cascade;
use drone_control::lqr::LqrController;
use drone_model::state::DroneState;
use drone_model::vehicle::quadrotor::QuadrotorModel;
use drone_model::vehicle::{F16Model, VehicleModel};
use drone_sitl::{
    runner::{ControllerFactory, run_scenario},
    scenario::{Scenario, VehicleKind},
};
use nalgebra::{UnitQuaternion, Vector3};
use std::path::PathBuf;

/// Build the vehicle model and a matching controller factory for the given
/// `VehicleKind`.  Using a factory closure (rather than a pre-built controller)
/// guarantees every scenario run starts from a clean controller state.
fn make_model_and_factory(vehicle: &VehicleKind) -> (Box<dyn VehicleModel>, ControllerFactory) {
    match vehicle {
        VehicleKind::QuadrotorMini3 => (
            Box::new(QuadrotorModel::mini3()),
            Box::new(|m| Ok(Box::new(make_cascade(m)))),
        ),
        VehicleKind::QuadrotorMini3Simple => (
            Box::new(QuadrotorModel::mini3_simple()),
            Box::new(|m| Ok(Box::new(make_cascade(m)))),
        ),
        VehicleKind::F16 => {
            // F-16 uses an LQR stabiliser designed around a cruise trim point.
            // The factory closure:
            //   1. Warms up the jet engine to steady-state thrust before the
            //      linearisation, so that the A/B matrices reflect real physics.
            //   2. Linearises around a near-trim state:
            //        h ≈ 0 m (sea level), V ≈ 200 m/s, α ≈ 5° (nose up)
            //      At these conditions the aerodynamic lift (≈ 89 kN) plus
            //      the thrust vertical component (≈ 3 kN) ≈ weight (91 kN).
            let factory: ControllerFactory = Box::new(|m| {
                use drone_model::time::TimeStep;

                // ── Step 1: warm up the jet engine ──────────────────────────
                // The JetEngine starts at zero thrust; the CARE diverges if we
                // linearise there.  We pre-step the actuators ≫ 5 × τ = 2.5 s
                // (τ = 0.5 s) so that thrust reaches the throttle-0.5 setpoint.
                let trim_input = m.equilibrium_input(); // throttle=0.5, elevator=-0.06
                let mut dummy = DroneState {
                    position: Vector3::new(0.0, 0.0, 0.0),
                    velocity: Vector3::new(200.0, 0.0, 0.0),
                    orientation: UnitQuaternion::identity(),
                    angular_velocity: Vector3::zeros(),
                    actuator_state: None,
                };
                let wdt = TimeStep::constant(0.01);
                for _ in 0..1000 { // 10 s >> 5 τ
                    m.step_actuators(&mut dummy, &trim_input, wdt);
                }

                // ── Step 2: trim state at α ≈ 5°, sea level, V = 200 m/s ───
                // At α = 0 with elevator = -0.06 the elevator contributes a
                // net DOWN force; the wing must operate at α ≈ 5° to produce
                // enough lift.  We represent the trim as:
                //   • horizontal velocity  [200, 0, 0]  (level flight)
                //   • nose-up orientation  pitch = -5°  (negative in nalgebra ENU)
                // With body pitched 5° up, v_body.z = -200 sin(5°) ≈ -17.4 m/s
                // → α = asin(-v_body.z / V) ≈ 5° in the aero model.
                // This exactly matches the initial conditions used in the F-16
                // scenario TOML files (attitude_deg = [0, -5, 0]).
                let alpha = 5.0_f64.to_radians();
                let trim_state = DroneState {
                    position: Vector3::zeros(),
                    velocity: Vector3::new(200.0, 0.0, 0.0), // horizontal
                    orientation: UnitQuaternion::from_euler_angles(0.0, -alpha, 0.0),
                    angular_velocity: Vector3::zeros(),
                    actuator_state: None,
                };

                // ── Step 3: LQR design ───────────────────────────────────────
                // Q weights: [pos xyz, vel xyz, ω xyz, quaternion xyzw] (13)
                let q_weights = vec![
                    0.1, 0.1, 50.0,            // x  y  z
                    1.0, 1.0,  5.0,            // vx vy vz
                    10.0, 10.0, 10.0,          // ωx ωy ωz
                    20.0, 20.0, 20.0, 20.0,   // qi qj qk qw
                ];
                let r_weights = vec![1.0, 0.5, 0.5, 1.0]; // throttle aileron elevator rudder
                let u_limits = vec![
                    (0.0, 1.0),   // throttle
                    (-1.0, 1.0),  // aileron
                    (-1.0, 1.0),  // elevator
                    (-1.0, 1.0),  // rudder
                ];
                LqrController::design(m, &trim_state, &q_weights, &r_weights, u_limits)
                    .map(|c| Box::new(c) as Box<dyn drone_control::controller::Controller>)
                    .map_err(|e| anyhow::anyhow!("F-16 LQR design failed: {}", e))
            });
            (Box::new(F16Model::f16a()), factory)
        }
    }
}

fn main() -> Result<()> {
    let scenarios_dir = PathBuf::from("scenarios");
    let mut passed = 0;
    let mut failed = 0;

    let mut entries: Vec<_> = std::fs::read_dir(&scenarios_dir)?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().map_or(false, |ext| ext == "toml"))
        .collect();

    entries.sort_by_key(|e| e.path());

    for entry in entries {
        let path = entry.path();
        let scenario = Scenario::from_file(&path)?;
        let (model, factory) = make_model_and_factory(&scenario.vehicle);
        let report = run_scenario(&scenario, model.as_ref(), &factory)?;
        report.print();

        if report.passed {
            passed += 1;
        } else {
            failed += 1;
        }
    }

    println!("\n═══════════════════════════════");
    println!("  Results: {} PASS, {} FAIL", passed, failed);
    println!("═══════════════════════════════\n");

    if failed > 0 {
        std::process::exit(1);
    }

    Ok(())
}
