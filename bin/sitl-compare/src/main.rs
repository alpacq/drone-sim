use anyhow::Result;
use clap::Parser;
use drone_model::vehicle::quadrotor::QuadrotorModel;
use drone_plot::{plot_comparison, plot_mpc_horizon};
use drone_sitl::{
    horizon::run_capturing_horizons,
    comparison::{ControllerFactory, compare_controllers},
    controller_config::{ControllerConfig, LqiConfig, LqrConfig, MpcConfig},
    scenario::Scenario,
};
use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::fmt::Write as FmtWrite;

// ── Configuration types ────────────────────────────────────────────────────

/// A named controller entry in a `compare.toml` config file.
///
/// The controller config lives in a nested `[config]` sub-table:
///
/// ```toml
/// [[controllers]]
/// name = "Cascade-default"
/// [controllers.config]
/// type = "cascade"
///
/// [[controllers]]
/// name = "LQR-aggressive"
/// [controllers.config]
/// type = "lqr"
/// trim_z_m = 5.0
/// q_weights = [1.0, 1.0, 100.0, 0.5, 0.5, 5.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0]
/// ```
#[derive(Debug, Clone, Deserialize)]
struct NamedController {
    name: String,
    config: ControllerConfig,
}

/// Top-level structure of a `--config compare.toml` file.
#[derive(Debug, Deserialize)]
struct CompareConfig {
    controllers: Vec<NamedController>,
}

impl CompareConfig {
    fn from_file(path: &Path) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        Ok(toml::from_str(&content)?)
    }

    /// The five controllers used in the default comparison.
    fn default_controllers() -> Vec<NamedController> {
        vec![
            NamedController {
                name: "Cascade-PID".into(),
                config: ControllerConfig::default(),
            },
            NamedController {
                name: "LQR-R=0.01".into(),
                config: ControllerConfig::Lqr(LqrConfig::default()),
            },
            NamedController {
                name: "LQR-R=1.0".into(),
                config: ControllerConfig::Lqr(LqrConfig {
                    r_weights: Some(vec![1.0; 4]),
                    ..LqrConfig::default()
                }),
            },
            NamedController {
                name: "LQI".into(),
                config: ControllerConfig::Lqi(LqiConfig::default()),
            },
            NamedController {
                name: "MPC-N=10".into(),
                config: ControllerConfig::Mpc(MpcConfig::default()),
            },
        ]
    }
}

// ── CLI ────────────────────────────────────────────────────────────────────

/// Compare multiple flight controllers side-by-side on SITL scenarios.
///
/// Runs each controller on the same set of scenarios and prints a table
/// with RMS error, settling time, control energy, and other metrics.
/// Without options runs the default four controllers on three scenarios.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// Load a comparison config from a TOML file.
    ///
    /// The file must contain a `[[controllers]]` array, each entry with a
    /// `name` field and a nested `[controller.config]` table.
    /// When omitted, the default four controllers are compared:
    /// Cascade-PID, LQR-R=0.01, LQR-R=1.0, LQI.
    #[arg(long, short = 'c')]
    config: Option<PathBuf>,

    /// Scenario TOML files to run, comma-separated.
    /// Defaults to step_response, disturbance_rejection, turbulence_comparison.
    #[arg(long, value_delimiter = ',')]
    scenarios: Option<Vec<PathBuf>>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // ── Controller list ────────────────────────────────────────────────────
    let named = if let Some(path) = &cli.config {
        CompareConfig::from_file(path)?.controllers
    } else {
        CompareConfig::default_controllers()
    };

    let model = QuadrotorModel::mini3_simple();

    // ── Scenario list ──────────────────────────────────────────────────────
    let default_scenarios = [
        PathBuf::from("scenarios/step_response.toml"),
        PathBuf::from("scenarios/disturbance_rejection.toml"),
        PathBuf::from("scenarios/turbulence_comparison.toml"),
    ];
    let scenario_paths: &[PathBuf] = cli.scenarios.as_deref().unwrap_or(&default_scenarios);

    // ── Run ────────────────────────────────────────────────────────────────
    let mut all_reports: Vec<(String, drone_sitl::comparison::ComparisonReport)> = Vec::new();

    for path in scenario_paths {
        if !path.exists() {
            println!("Skipped (file not found): {}", path.display());
            continue;
        }

        let scenario = Scenario::from_file(path)?;
        println!("\nRunning: {}", scenario.name);

        // Build a fresh set of factories for each scenario so the CARE solver
        // starts from a clean state.  ControllerConfig is Clone, so the same
        // config can be reused across multiple scenario runs.
        let factories: Vec<(&str, ControllerFactory)> = named
            .iter()
            .map(|nc| (nc.name.as_str(), nc.config.clone().into_factory()))
            .collect();

        let report = compare_controllers(&scenario, &model, &factories)?;
        report.print();

        std::fs::create_dir_all("target")?;

        // Save trajectories to CSV
        let csv_path = format!("target/{}_trajectories.csv", scenario.name);
        std::fs::write(&csv_path, report.trajectories_to_csv())?;
        println!("  Trajectories saved: {}", csv_path);

        // Save metrics
        let metrics_path = format!("target/{}_metrics.csv", scenario.name);
        std::fs::write(&metrics_path, report.to_csv())?;
        println!("  Metrics saved: {}", metrics_path);

        // Generate plots
        match plot_comparison(&report, std::path::Path::new("target")) {
            Ok(()) => println!("  Plots saved: target/{}_trajectories.png, target/{}_metrics.png",
                               scenario.name, scenario.name),
            Err(e) => eprintln!("  Warning: could not generate plots: {}", e),
        }

        // MPC horizon plot: if MPC is among the controllers, run a dedicated
        // simulation that captures the planned trajectory at each step.
        let mpc_entry = named.iter().find(|nc| {
            matches!(nc.config, drone_sitl::controller_config::ControllerConfig::Mpc(_))
        });
        if let Some(mpc_nc) = mpc_entry {
            let mpc_factory = mpc_nc.config.clone().into_factory();
            let mpc_dt_s = match &mpc_nc.config {
                drone_sitl::controller_config::ControllerConfig::Mpc(c) => c.dt_s,
                _ => 0.5,
            };
            match run_capturing_horizons(
                &scenario,
                &model,
                &mpc_factory,
                1.0,       // snapshot every 1 second of sim time
                mpc_dt_s,  // MPC internal prediction step
            ) {
                Ok((frames, snapshots)) => {
                    match plot_mpc_horizon(
                        &frames,
                        &snapshots,
                        scenario.target.z,
                        &scenario.name,
                        std::path::Path::new("target"),
                    ) {
                        Ok(()) => println!(
                            "  MPC horizon plot: target/{}_mpc_horizon.png  ({} snapshots)",
                            scenario.name,
                            snapshots.len()
                        ),
                        Err(e) => eprintln!("  Warning: MPC horizon plot failed: {}", e),
                    }
                }
                Err(e) => eprintln!("  Warning: MPC horizon capture failed: {}", e),
            }
        }

        all_reports.push((scenario.name.clone(), report));
    }

    // Write markdown report
    let report_path = format!(
        "target/report_{}.md",
        chrono::Local::now().format("%Y-%m-%d_%H-%M")
    );
    let report_md = build_comparison_report(&all_reports);
    match std::fs::write(&report_path, &report_md) {
        Ok(()) => println!("\nReport saved: {}", report_path),
        Err(e) => eprintln!("Warning: could not write report: {}", e),
    }

    Ok(())
}

// ── Markdown report ───────────────────────────────────────────────────────────

fn build_comparison_report(
    all_reports: &[(String, drone_sitl::comparison::ComparisonReport)],
) -> String {
    let mut md = String::new();
    let _ = writeln!(md, "# drone-sim — Controller Comparison Report");
    let _ = writeln!(md, "");
    let _ = writeln!(
        md,
        "Generated: {}",
        chrono::Local::now().format("%Y-%m-%d %H:%M:%S")
    );
    let _ = writeln!(md, "");

    for (name, report) in all_reports {
        let _ = writeln!(md, "## Scenario: `{}`", name);
        let _ = writeln!(md, "");
        let _ = writeln!(
            md,
            "| Controller | RMS Z [m] | OS [%] | ST [s] | RT [s] | Energy |"
        );
        let _ = writeln!(
            md,
            "|---|---|---|---|---|---|"
        );
        for r in &report.results {
            let _ = writeln!(
                md,
                "| {} | {:.3} | {:.1} | {:.2} | {:.2} | {:.0} |",
                r.name,
                r.rms_error_z,
                r.overshoot_pct,
                r.settling_time_s,
                r.rise_time_s,
                r.control_energy,
            );
        }
        let _ = writeln!(md, "");
        let _ = writeln!(
            md,
            "**Plots:** [`{n}_trajectories.png`](target/{n}_trajectories.png) · \
             [`{n}_metrics.png`](target/{n}_metrics.png) · \
             [`{n}_mpc_horizon.png`](target/{n}_mpc_horizon.png)",
            n = name
        );
        let _ = writeln!(md, "");
    }

    md
}
