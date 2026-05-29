use anyhow::Result;
use drone_sitl::report::ScenarioReport;
use plotters::prelude::*;
use std::path::Path;

use crate::palette::PALETTE;
use crate::slug;

/// Generuje `{dir}/{name}_step_response.png`:
/// linia z(t) w kolorze zielonym (PASS) lub czerwonym (FAIL)
/// z szarą linią celu.
pub fn plot_scenario(report: &ScenarioReport, target_z: f64, out_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(out_dir)?;

    let path = out_dir.join(format!("{}_step_response.png", slug(&report.name)));
    let root = BitMapBackend::new(&path, (1024, 480)).into_drawing_area();
    root.fill(&WHITE)?;

    let t_max = report.frames.last().map(|f| f.time).unwrap_or(1.0);

    let z_vals: Vec<f64> = report
        .frames
        .iter()
        .map(|f| f.state.position.z)
        .chain(std::iter::once(target_z))
        .collect();
    let z_min = z_vals.iter().cloned().fold(f64::INFINITY, f64::min);
    let z_max = z_vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let z_pad = ((z_max - z_min) * 0.12).max(0.3);

    let status = if report.passed { "PASS ✓" } else { "FAIL ✗" };
    let color = if report.passed { PALETTE[2] } else { PALETTE[3] };

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("{} [{}]", report.name, status),
            ("sans-serif", 18),
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

    // Linia celu
    chart
        .draw_series(LineSeries::new(
            [(0.0, target_z), (t_max, target_z)],
            BLACK.mix(0.35).stroke_width(2),
        ))?
        .label(format!("target ({:.1} m)", target_z))
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], BLACK.mix(0.35)));

    // Trajektoria
    chart
        .draw_series(LineSeries::new(
            report.frames.iter().map(|f| (f.time, f.state.position.z)),
            color.stroke_width(2),
        ))?
        .label("altitude")
        .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color));

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}
