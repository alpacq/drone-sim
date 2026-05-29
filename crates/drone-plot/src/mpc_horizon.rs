use anyhow::Result;
use drone_sim::runner::SimFrame;
use drone_sitl::horizon::HorizonSnapshot;
use plotters::prelude::*;
use std::path::Path;

use crate::palette::PALETTE;
use crate::slug;

/// Generuje `{dir}/{name}_mpc_horizon.png`.
///
/// # Zawartość wykresu
/// * **Szara linia przerywana** — cel (target z)
/// * **Niebieska linia ciągła** — rzeczywista trajektoria z(t)
/// * **Pomarańczowe linie** — plany predykcyjne MPC co ~1 s, pokazujące
///   gdzie kontroler "planował" być w ciągu najbliższych N kroków.
///   Im jaśniejszy odcień, tym dalszy horyzont predykcji.
pub fn plot_mpc_horizon(
    frames: &[SimFrame],
    snapshots: &[HorizonSnapshot],
    target_z: f64,
    scenario_name: &str,
    out_dir: &Path,
) -> Result<()> {
    std::fs::create_dir_all(out_dir)?;

    let path = out_dir.join(format!("{}_mpc_horizon.png", slug(scenario_name)));
    let root = BitMapBackend::new(&path, (1100, 560)).into_drawing_area();
    root.fill(&WHITE)?;

    let t_max = frames.last().map(|f| f.time).unwrap_or(1.0);

    // Collect all z values (actual + predictions) for y-axis range
    let mut all_z: Vec<f64> = frames.iter().map(|f| f.state.position.z).collect();
    all_z.push(target_z);
    for s in snapshots {
        all_z.extend_from_slice(&s.pred_z);
    }
    let z_min = all_z.iter().cloned().fold(f64::INFINITY, f64::min);
    let z_max = all_z.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let z_pad = ((z_max - z_min) * 0.12).max(0.5);

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("MPC receding-horizon plan — {}", scenario_name),
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

    // ── Target line ──────────────────────────────────────────────────────────

    chart
        .draw_series(LineSeries::new(
            [(0.0, target_z), (t_max, target_z)],
            BLACK.mix(0.35).stroke_width(2),
        ))?
        .label(format!("target ({:.1} m)", target_z))
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], BLACK.mix(0.35)));

    // ── Horizon predictions (drawn before actual, so actual appears on top) ──
    //
    // Each snapshot is a thin light-orange line from (sim_time, z_current) to
    // (sim_time + k·pred_dt, z_pred[k]).  Use alpha mixing to distinguish
    // near-term (darker) from far-term (lighter) predictions.

    let horizon_color = RGBColor(0xff, 0x7f, 0x0e); // matplotlib orange

    for snap in snapshots {
        if snap.pred_z.is_empty() {
            continue;
        }
        let pts: Vec<(f64, f64)> = snap
            .pred_z
            .iter()
            .enumerate()
            .map(|(k, &z)| (snap.pred_time(k), z))
            .collect();

        chart.draw_series(LineSeries::new(pts, horizon_color.mix(0.45).stroke_width(1)))?;
    }

    // Draw one dummy series entry so the legend shows "MPC plan"
    chart
        .draw_series(LineSeries::new(
            [(-1.0, 0.0), (-1.0, 0.0)], // invisible stub
            horizon_color.mix(0.5).stroke_width(1),
        ))?
        .label("MPC plan (per 1 s)")
        .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], horizon_color.mix(0.5)));

    // ── Actual trajectory ────────────────────────────────────────────────────

    chart
        .draw_series(LineSeries::new(
            frames.iter().map(|f| (f.time, f.state.position.z)),
            PALETTE[0].stroke_width(2),
        ))?
        .label("actual z(t)")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], PALETTE[0]));

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}
