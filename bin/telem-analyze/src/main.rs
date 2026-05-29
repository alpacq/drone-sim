use anyhow::Result;
use clap::Parser;
use drone_analysis::{VALID_POSITION_THRESHOLD_M, ValidateConfig, validate_model};
use drone_model::{time::TimeStep, vehicle::quadrotor::QuadrotorModel};
use drone_plot::plot_validation;
use drone_telemetry::{normalize, parse_file};
use std::path::PathBuf;

/// Validate a physical drone model against a DJI SRT telemetry file.
///
/// Parses the SRT subtitles, normalises GPS points into ENU coordinates,
/// runs an open-loop simulation of the Mini 3 model, and reports
/// position / velocity error metrics.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// DJI .srt file to analyse.  Defaults to data/DJI_0001.srt.
    #[arg(default_value = "data/DJI_0001.srt")]
    file: PathBuf,

    /// Simulation time step [s].  Defaults to the mean telemetry frame
    /// interval (inverse of the SRT frame rate).
    #[arg(long, short = 'd')]
    dt_s: Option<f64>,

    /// Position-error threshold for the "model valid until t" metric [m].
    /// A timestamp is counted as valid as long as |pos_error| < threshold.
    #[arg(long, default_value_t = VALID_POSITION_THRESHOLD_M)]
    threshold_m: f64,

    /// Write the per-point comparison table to a CSV file next to the input.
    #[arg(long, short = 'o')]
    save_csv: bool,

    /// Generate a validation PNG plot in target/.
    #[arg(long)]
    plot: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // ── Telemetry ────────────────────────────────────────────────────────────
    println!("Loading: {}", cli.file.display());

    let frames = parse_file(&cli.file)?;
    println!("  {} SRT frames parsed", frames.len());

    let traj = normalize(&frames)?;
    println!(
        "  {:.1}s flight  •  {} GPS points",
        traj.duration_s,
        traj.len()
    );

    let max_alt_m = traj
        .points
        .iter()
        .map(|p| p.position.z)
        .fold(0.0_f64, f64::max);
    let max_speed_ms = traj
        .points
        .iter()
        .filter_map(|p| p.velocity)
        .map(|v| v.norm())
        .fold(0.0_f64, f64::max);
    println!("  Max altitude:  {:.1} m", max_alt_m);
    println!(
        "  Max speed:     {:.1} m/s  ({:.1} km/h)",
        max_speed_ms,
        max_speed_ms * 3.6
    );

    // ── Model validation ───────────────────────────────────────────────────
    let model = QuadrotorModel::mini3();

    // Infer simulation dt from the telemetry frame rate when not specified.
    // The mean frame interval keeps the simulation aligned with the data.
    let dt_s = cli
        .dt_s
        .unwrap_or_else(|| traj.duration_s / traj.len() as f64);
    let dt = TimeStep::new(dt_s).unwrap_or_else(|_| TimeStep::constant(0.033)); // 30 fps fallback

    let config = ValidateConfig {
        dt,
        valid_position_threshold_m: cli.threshold_m,
    };

    let source_name = cli
        .file
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("unknown")
        .to_string();

    let report = validate_model(&model, &traj, config, source_name)?;
    report.print();

    // ── CSV export ──────────────────────────────────────────────────────────
    if cli.save_csv {
        let csv_path = cli.file.with_extension("validation.csv");
        std::fs::write(&csv_path, report.to_csv())?;
        println!("Results saved: {}", csv_path.display());
    }

    // ── PNG plot ─────────────────────────────────────────────────────────────
    if cli.plot {
        std::fs::create_dir_all("target").ok();
        match plot_validation(&report, std::path::Path::new("target")) {
            Ok(()) => println!("Plot saved in target/"),
            Err(e) => eprintln!("Warning: could not generate plot: {}", e),
        }
    }

    Ok(())
}
