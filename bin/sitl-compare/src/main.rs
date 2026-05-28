use anyhow::Result;
use drone_control::cascade::make_cascade;
use drone_control::lqr::{LqiController, LqrController, quadrotor_c_integral};
use drone_model::vehicle::quadrotor::QuadrotorModel;
use drone_sitl::{
    comparison::{ControllerFactory, compare_controllers},
    scenario::Scenario,
};
use std::path::Path;

fn main() -> Result<()> {
    let model = QuadrotorModel::mini3_simple();
    let pid_factory: ControllerFactory = Box::new(|model| Ok(Box::new(make_cascade(model))));
    let lqr_factory: ControllerFactory = Box::new(|model| {
        use drone_model::state::DroneState;
        use nalgebra::{UnitQuaternion, Vector3};

        let trim_state = DroneState {
            position: Vector3::new(0.0, 0.0, 5.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };

        let q_weights = vec![
            1.0, 1.0, 50.0, 0.5, 0.5, 5.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0,
        ];

        let r_weights = vec![0.01; 4];

        let hover_w = match model.equilibrium_input() {
            drone_model::vehicle::KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => anyhow::bail!("Unexpected input type"),
        };
        let u_limits = vec![(0.0, hover_w * 2.0); 4];

        let ctrl = LqrController::design(model, &trim_state, &q_weights, &r_weights, u_limits)?;

        Ok(Box::new(ctrl))
    });

    let lqr_smooth_factory: ControllerFactory = Box::new(|model| {
        use drone_model::state::DroneState;
        use nalgebra::{UnitQuaternion, Vector3};

        let trim_state = DroneState {
            position: Vector3::new(0.0, 0.0, 5.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };

        let q_weights = vec![
            1.0, 1.0, 50.0, 0.5, 0.5, 5.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0,
        ];
        let r_weights = vec![1.0; 4];

        let hover_w = match model.equilibrium_input() {
            drone_model::vehicle::KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => anyhow::bail!("Unexpected type"),
        };
        let u_limits = vec![(0.0, hover_w * 2.0); 4];

        Ok(Box::new(LqrController::design(
            model,
            &trim_state,
            &q_weights,
            &r_weights,
            u_limits,
        )?))
    });

    // ── LQI factory ─────────────────────────────────────────────────────────
    // 13 plant weights + 4 integral weights [ξ_x, ξ_y, ξ_z, ξ_ψ]
    let lqi_factory: ControllerFactory = Box::new(|model| {
        use drone_model::state::DroneState;
        use nalgebra::{UnitQuaternion, Vector3};

        let trim_state = DroneState {
            position: Vector3::new(0.0, 0.0, 5.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };

        let q_weights = vec![
            // plant: xyz, vxyz, ωxyz, quaternion
            1.0, 1.0, 50.0,  0.5, 0.5, 5.0,  2.0, 2.0, 2.0,  20.0, 20.0, 20.0, 20.0,
            // integrals: ξ_x  ξ_y  ξ_z  ξ_ψ
            5.0, 5.0, 30.0, 2.0,
        ];
        let r_weights = vec![0.01; 4];

        let hover_w = match model.equilibrium_input() {
            drone_model::vehicle::KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => anyhow::bail!("Unexpected input type"),
        };
        let u_limits = vec![(0.0, hover_w * 2.0); 4];

        let c_int = quadrotor_c_integral(13);
        Ok(Box::new(LqiController::design(
            model, &trim_state, c_int, &q_weights, &r_weights, u_limits,
        )?))
    });

    let factories: Vec<(&str, ControllerFactory)> = vec![
        ("PID-Cascade", pid_factory),
        ("LQR-R=0.01", lqr_factory),
        ("LQR-R=1.0", lqr_smooth_factory),
        ("LQI", lqi_factory),
    ];

    // ── Uruchom porównanie na każdym scenariuszu ─────────────────
    let scenario_files = [
        "scenarios/step_response.toml",
        "scenarios/disturbance_rejection.toml",
        "scenarios/turbulence_comparison.toml",
    ];

    for path in &scenario_files {
        if !Path::new(path).exists() {
            println!("Missed (no file): {}", path);
            continue;
        }

        let scenario = Scenario::from_file(Path::new(path))?;
        println!("\nRunning: {}", scenario.name);

        let report = compare_controllers(&scenario, &model, &factories)?;
        report.print();

        // Zapisz trajektorie do CSV
        let csv_path = format!("target/{}_trajectories.csv", scenario.name);
        std::fs::create_dir_all("target")?;
        std::fs::write(&csv_path, report.trajectories_to_csv())?;
        println!("  Trajectories saved: {}", csv_path);

        // Zapisz metryki
        let metrics_path = format!("target/{}_metrics.csv", scenario.name);
        std::fs::write(&metrics_path, report.to_csv())?;
        println!("  Metrics saved: {}", metrics_path);
    }

    Ok(())
}
