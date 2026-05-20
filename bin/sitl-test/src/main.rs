use anyhow::Result;
use drone_model::params::DroneParams;
use drone_sitl::{runner::run_scenario, scenario::Scenario};
use std::path::PathBuf;

fn main() -> Result<()> {
    let params = DroneParams::mini3();

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
        let report = run_scenario(&scenario, &params)?;
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
