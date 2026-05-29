use clap::Parser;
use drone_model::vehicle::quadrotor::QuadrotorModel;
use drone_plot::plot_monte_carlo;
use drone_sitl::{
    controller_config::{CascadeConfig, ControllerConfig, LqiConfig, LqrConfig},
    monte_carlo::{run_monte_carlo, MonteCarloConfig},
    scenario::Scenario,
};
use std::path::PathBuf;

/// Monte Carlo batch simulation — runs a scenario many times with perturbed
/// initial conditions and reports per-metric statistics.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// Path to the scenario TOML file.
    #[arg(long, short = 's')]
    scenario: PathBuf,

    /// Number of independent runs.
    #[arg(long, default_value_t = 100)]
    runs: usize,

    /// Position noise std dev [m].
    #[arg(long, default_value_t = 0.5)]
    pos_noise: f64,

    /// Velocity noise std dev [m/s].
    #[arg(long, default_value_t = 0.1)]
    vel_noise: f64,

    /// RNG seed.
    #[arg(long, default_value_t = 42)]
    seed: u64,

    /// Controller kind (cascade | lqr | lqi).
    #[arg(long, value_enum, default_value_t = ControllerKind::Cascade)]
    controller: ControllerKind,

    /// Path to controller config TOML.
    #[arg(long, short = 'c')]
    config: Option<PathBuf>,
}

#[derive(Debug, Clone, clap::ValueEnum, Default)]
enum ControllerKind {
    #[default]
    Cascade,
    Lqr,
    Lqi,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    let ctrl_cfg: ControllerConfig = if let Some(p) = &cli.config {
        ControllerConfig::from_file(p)?
    } else {
        match cli.controller {
            ControllerKind::Cascade => ControllerConfig::Cascade(CascadeConfig::default()),
            ControllerKind::Lqr => ControllerConfig::Lqr(LqrConfig::default()),
            ControllerKind::Lqi => ControllerConfig::Lqi(LqiConfig::default()),
        }
    };

    let scenario = Scenario::from_file(&cli.scenario)?;
    let model = QuadrotorModel::mini3();
    let factory = ctrl_cfg.into_factory();

    let cfg = MonteCarloConfig {
        runs: cli.runs,
        pos_noise_m: cli.pos_noise,
        vel_noise_ms: cli.vel_noise,
        base_seed: cli.seed,
    };

    println!("Running {} Monte Carlo iterations...", cfg.runs);
    let report = run_monte_carlo(&scenario, &model, &factory, &cfg);
    report.print();

    // Save CSV statistics
    std::fs::create_dir_all("target")?;
    let csv_path = format!("target/{}_mc.csv", scenario.name);
    std::fs::write(&csv_path, report.to_csv())?;
    println!("Statistics saved: {}", csv_path);

    // Generate plot
    match plot_monte_carlo(&report, std::path::Path::new("target")) {
        Ok(()) => println!("Plot saved:       target/{}_mc.png", scenario.name),
        Err(e) => eprintln!("Warning: could not generate plot: {}", e),
    }

    Ok(())
}
