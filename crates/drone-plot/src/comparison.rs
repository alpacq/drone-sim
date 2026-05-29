use anyhow::Result;
use drone_sitl::comparison::ComparisonReport;
use plotters::prelude::*;
use std::path::Path;

use crate::palette::PALETTE;
use crate::slug;

const W: u32 = 1024;
const H_TRAJ: u32 = 520;
const H_METRICS: u32 = 500;

/// Generuje dwa pliki PNG dla raportu porównawczego:
/// * `{dir}/{name}_trajectories.png` – linie z(t) per regulator
/// * `{dir}/{name}_metrics.png`      – siatka 2×2 słupków kluczowych metryk
pub fn plot_comparison(report: &ComparisonReport, out_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(out_dir)?;
    plot_trajectories(report, out_dir)?;
    plot_metrics(report, out_dir)?;
    Ok(())
}

// ── Wykres trajektorii ───────────────────────────────────────────────────────

fn plot_trajectories(report: &ComparisonReport, out_dir: &Path) -> Result<()> {
    let path = out_dir.join(format!("{}_trajectories.png", slug(&report.scenario_name)));
    let root = BitMapBackend::new(&path, (W, H_TRAJ)).into_drawing_area();
    root.fill(&WHITE)?;

    // Zakresy osi
    let t_max = report
        .results
        .iter()
        .flat_map(|r| r.frames.iter().map(|f| f.time))
        .fold(1.0_f64, f64::max);

    let z_vals: Vec<f64> = report
        .results
        .iter()
        .flat_map(|r| r.frames.iter().map(|f| f.state.position.z))
        .chain(std::iter::once(report.target_z))
        .collect();
    let z_min = z_vals.iter().cloned().fold(f64::INFINITY, f64::min);
    let z_max = z_vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let z_pad = ((z_max - z_min) * 0.12).max(0.3);

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("Altitude — {}", report.scenario_name),
            ("sans-serif", 20),
        )
        .margin(14)
        .x_label_area_size(38)
        .y_label_area_size(54)
        .build_cartesian_2d(0f64..t_max, (z_min - z_pad)..(z_max + z_pad))?;

    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Altitude [m]")
        .draw()?;

    // Linia celu (szara, przerywana imitowana zmianą alpha)
    chart
        .draw_series(LineSeries::new(
            [(0.0, report.target_z), (t_max, report.target_z)],
            BLACK.mix(0.35).stroke_width(2),
        ))?
        .label("target")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], BLACK.mix(0.35)));

    // Linie per regulator
    for (i, res) in report.results.iter().enumerate() {
        let color = PALETTE[i % PALETTE.len()];
        chart
            .draw_series(LineSeries::new(
                res.frames.iter().map(|f| (f.time, f.state.position.z)),
                color.stroke_width(2),
            ))?
            .label(res.name.clone())
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color));
    }

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}

// ── Wykres metryk (siatka 2×2) ───────────────────────────────────────────────

fn plot_metrics(report: &ComparisonReport, out_dir: &Path) -> Result<()> {
    let path = out_dir.join(format!("{}_metrics.png", slug(&report.scenario_name)));
    let root = BitMapBackend::new(&path, (W, H_METRICS)).into_drawing_area();
    root.fill(&WHITE)?;

    let areas = root.split_evenly((2, 2));
    let n = report.results.len();
    let names: Vec<&str> = report.results.iter().map(|r| r.name.as_str()).collect();

    // Cztery metryki do porównania
    let metrics: [(&str, Vec<f64>); 4] = [
        (
            "RMS error Z [m]",
            report.results.iter().map(|r| r.rms_error_z).collect(),
        ),
        (
            "Overshoot [%]",
            report.results.iter().map(|r| r.overshoot_pct).collect(),
        ),
        (
            "Settling time [s]",
            report.results.iter().map(|r| r.settling_time_s).collect(),
        ),
        (
            "Control energy",
            report.results.iter().map(|r| r.control_energy).collect(),
        ),
    ];

    for (area, (title, values)) in areas.iter().zip(metrics.iter()) {
        let max_val = values.iter().cloned().fold(1e-6_f64, f64::max);
        let y_max = max_val * 1.3;

        let mut chart = ChartBuilder::on(area)
            .caption(*title, ("sans-serif", 15))
            .margin(8)
            .x_label_area_size(30)
            .y_label_area_size(50)
            .build_cartesian_2d(0f64..n as f64, 0f64..y_max)?;

        chart
            .configure_mesh()
            .disable_x_mesh()
            .x_labels(n)
            .x_label_formatter(&|x: &f64| {
                let i = (*x).round() as usize;
                names
                    .get(i)
                    .map(|s| s.chars().take(8).collect::<String>())
                    .unwrap_or_default()
            })
            .draw()?;

        chart.draw_series(values.iter().enumerate().map(|(i, &v)| {
            let color = PALETTE[i % PALETTE.len()];
            Rectangle::new([(i as f64, 0.0), (i as f64 + 0.8, v)], color.filled())
        }))?;
    }

    root.present()?;
    Ok(())
}
