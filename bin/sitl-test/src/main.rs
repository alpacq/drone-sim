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

/// LQR gain weights and actuator limits for the F-16 cruise trim.
///
/// Separating these constants from the factory closure makes the
/// design intent clear and simplifies tuning.
struct F16LqrConfig {
    /// Number of engine warm-up steps (each at dt_s).  10 s >> 5τ (0.5 s).
    warmup_steps: usize,
    warmup_dt_s: f64,
    /// Trim angle of attack [deg] — matches the F-16 scenario TOML `attitude_deg`.
    trim_alpha_deg: f64,
    /// Trim airspeed [m/s] at sea level.
    trim_speed_ms: f64,
    /// Q weights: [pos xyz, vel xyz, ω xyz, quaternion xyzw] — 13 total.
    q: Vec<f64>,
    /// R weights: [throttle, aileron, elevator, rudder].
    r: Vec<f64>,
    /// Actuator limits: [(lo, hi); 4].
    u_limits: Vec<(f64, f64)>,
}

impl F16LqrConfig {
    /// Sensible defaults for level cruise near sea level at 200 m/s, α = 5°.
    fn cruise() -> Self {
        Self {
            warmup_steps: 1_000,
            warmup_dt_s:  0.01,
            trim_alpha_deg: 5.0,
            trim_speed_ms:  200.0,
            q: vec![
                0.1, 0.1, 50.0,          // x  y  z
                1.0, 1.0,  5.0,          // vx vy vz
                10.0, 10.0, 10.0,        // ωx ωy ωz
                20.0, 20.0, 20.0, 20.0, // qi qj qk qw
            ],
            r: vec![1.0, 0.5, 0.5, 1.0], // throttle aileron elevator rudder
            u_limits: vec![
                (0.0,  1.0), // throttle [0, 1]
                (-1.0, 1.0), // aileron
                (-1.0, 1.0), // elevator
                (-1.0, 1.0), // rudder
            ],
        }
    }
}

/// Build a `ControllerFactory` that designs an LQR stabiliser for the F-16
/// around the given `cfg` trim point.
///
/// The factory:
///  1. Warms up the jet engine so the linearisation reflects real thrust.
///  2. Builds the trim `DroneState` at `alpha = cfg.trim_alpha_deg`.
///  3. Calls `LqrController::design` and boxes the result.
fn f16_lqr_factory(cfg: F16LqrConfig) -> ControllerFactory {
    Box::new(move |m| {
        use drone_model::time::TimeStep;

        // Step 1: warm up the jet engine.
        // JetEngine starts at zero thrust; CARE diverges without warm-up.
        let trim_input = m.equilibrium_input(); // throttle=0.5, elevator=-0.06
        let mut dummy = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::new(cfg.trim_speed_ms, 0.0, 0.0),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };
        let wdt = TimeStep::constant(cfg.warmup_dt_s);
        for _ in 0..cfg.warmup_steps {
            m.step_actuators(&mut dummy, &trim_input, wdt);
        }

        // Step 2: trim state.
        // Horizontal velocity + pitched body gives AoA ≈ trim_alpha_deg in
        // the aero model (v_body.z = -V·sin(α) < 0 → α > 0).
        let alpha = cfg.trim_alpha_deg.to_radians();
        let trim_state = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::new(cfg.trim_speed_ms, 0.0, 0.0),
            orientation: UnitQuaternion::from_euler_angles(0.0, -alpha, 0.0),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };

        // Step 3: LQR design.
        LqrController::design(m, &trim_state, &cfg.q, &cfg.r, cfg.u_limits.clone())
            .map(|c| Box::new(c) as Box<dyn drone_control::controller::Controller>)
            .map_err(|e| anyhow::anyhow!("F-16 LQR design failed: {}", e))
    })
}

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
            let factory = f16_lqr_factory(F16LqrConfig::cruise());
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
        .filter(|e| e.path().extension().is_some_and(|ext| ext == "toml"))
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
