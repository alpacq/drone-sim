use anyhow::Result;
use drone_analysis::{ValidationReport, VALID_POSITION_THRESHOLD_M};
use plotters::prelude::*;
use std::path::Path;

use crate::palette::PALETTE;
use crate::slug;

/// Generuje `{dir}/{stem}_validation.png` z dwoma panelami:
/// * Górny – wysokość symulacji vs telemetrii
/// * Dolny  – błąd 3D pozycji z linią progu walidacji
pub fn plot_validation(report: &ValidationReport, out_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(out_dir)?;

    // Nazwa pliku na podstawie źródła telemetrii
    let stem = report
        .source_file
        .trim_end_matches(".srt")
        .trim_end_matches(".csv")
        .trim_end_matches(".SRT")
        .trim_end_matches(".CSV");
    let path = out_dir.join(format!("{}_validation.png", slug(stem)));

    let root = BitMapBackend::new(&path, (1024, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    // Podziel na górny (60%) i dolny (40%) panel
    let (top, bottom) = root.split_vertically(360);

    let pts = &report.trajectory.points;
    let t_max = report.trajectory.duration_s.max(1.0);

    // ── Panel górny: sim_z vs telem_z ────────────────────────────────────────

    let z_vals: Vec<f64> = pts
        .iter()
        .flat_map(|p| [p.sim_pos.z, p.telem_pos.z])
        .collect();
    let z_min = z_vals.iter().cloned().fold(f64::INFINITY, f64::min);
    let z_max = z_vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let z_pad = ((z_max - z_min) * 0.12).max(0.5);

    let mut chart_top = ChartBuilder::on(&top)
        .caption(
            format!("Model validation — {}", report.source_file),
            ("sans-serif", 17),
        )
        .margin(10)
        .x_label_area_size(0) // skryjemy dolne etykiety – są w panelu dolnym
        .y_label_area_size(52)
        .build_cartesian_2d(0f64..t_max, (z_min - z_pad)..(z_max + z_pad))?;

    chart_top
        .configure_mesh()
        .y_desc("Altitude [m]")
        .draw()?;

    chart_top
        .draw_series(LineSeries::new(
            pts.iter().map(|p| (p.time, p.sim_pos.z)),
            PALETTE[0].stroke_width(2),
        ))?
        .label("simulation")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], PALETTE[0]));

    chart_top
        .draw_series(LineSeries::new(
            pts.iter().map(|p| (p.time, p.telem_pos.z)),
            PALETTE[1].stroke_width(2),
        ))?
        .label("telemetry")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], PALETTE[1]));

    chart_top
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    // ── Panel dolny: błąd 3D pozycji ─────────────────────────────────────────

    let err_max = pts
        .iter()
        .map(|p| p.pos_error)
        .fold(0.0_f64, f64::max);
    let err_y_max = (err_max * 1.25).max(VALID_POSITION_THRESHOLD_M * 1.5);

    let mut chart_bot = ChartBuilder::on(&bottom)
        .margin(10)
        .x_label_area_size(38)
        .y_label_area_size(52)
        .build_cartesian_2d(0f64..t_max, 0f64..err_y_max)?;

    chart_bot
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Position error [m]")
        .draw()?;

    chart_bot
        .draw_series(LineSeries::new(
            pts.iter().map(|p| (p.time, p.pos_error)),
            PALETTE[2].stroke_width(2),
        ))?
        .label("3D error")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], PALETTE[2]));

    // Próg walidacji
    chart_bot
        .draw_series(LineSeries::new(
            [
                (0.0, VALID_POSITION_THRESHOLD_M),
                (t_max, VALID_POSITION_THRESHOLD_M),
            ],
            RED.mix(0.65).stroke_width(2),
        ))?
        .label(format!("threshold ({:.1} m)", VALID_POSITION_THRESHOLD_M))
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], RED.mix(0.65)));

    chart_bot
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}
