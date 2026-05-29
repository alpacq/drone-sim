use anyhow::Result;
use drone_sitl::monte_carlo::{MetricStats, MonteCarloReport};
use plotters::prelude::*;
use std::path::Path;

use crate::slug;

/// Generuje `{dir}/{name}_mc.png`.
///
/// Każda metryka pokazana jako „box plot" znormalizowany przez próg:
/// * Szary pasek: zakres min–max
/// * Niebieski pasek: przedział mean ± std_dev
/// * Ciemna linia pozioma: wartość mean
/// * Czerwona linia pozioma: próg (threshold), zawsze na y = 1.0
/// * Adnotacja z pass_rate [%]
pub fn plot_monte_carlo(report: &MonteCarloReport, out_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(out_dir)?;

    let n = report.metrics.len();
    if n == 0 {
        return Ok(());
    }

    let path = out_dir.join(format!("{}_mc.png", slug(&report.scenario_name)));
    let root = BitMapBackend::new(&path, (1024, 480)).into_drawing_area();
    root.fill(&WHITE)?;

    // Normalizuj wartości do threshold (threshold = 1.0 na osi Y)
    let norm = |m: &MetricStats, v: f64| -> f64 {
        if m.threshold > 1e-12 {
            v / m.threshold
        } else {
            v
        }
    };

    let y_max = report
        .metrics
        .iter()
        .map(|m| norm(m, m.max))
        .fold(1.5_f64, f64::max)
        * 1.15;

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!(
                "Monte Carlo — {}  ({} runs)",
                report.scenario_name, report.runs
            ),
            ("sans-serif", 19),
        )
        .margin(15)
        .x_label_area_size(52)
        .y_label_area_size(56)
        .build_cartesian_2d(0f64..n as f64, 0f64..y_max)?;

    chart
        .configure_mesh()
        .disable_x_mesh()
        .y_desc("Value / Threshold")
        .x_labels(n)
        .x_label_formatter(&|x: &f64| {
            let i = (*x).round() as usize;
            report
                .metrics
                .get(i)
                .map(|m| m.name.chars().take(14).collect::<String>())
                .unwrap_or_default()
        })
        .draw()?;

    // Czerwona linia threshold = 1.0
    chart
        .draw_series(LineSeries::new(
            [(0.0, 1.0), (n as f64, 1.0)],
            RED.mix(0.7).stroke_width(2),
        ))?
        .label("threshold")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 20, y)], RED.mix(0.7)));

    for (i, m) in report.metrics.iter().enumerate() {
        let x_l = i as f64 + 0.06;
        let x_r = i as f64 + 0.74;

        // Zakres min–max: jasnoszary
        chart.draw_series(std::iter::once(Rectangle::new(
            [
                (x_l + 0.08, norm(m, m.min)),
                (x_r - 0.08, norm(m, m.max).min(y_max)),
            ],
            RGBColor(190, 190, 190).filled(),
        )))?;

        // mean ± std_dev: niebieski półprzezroczysty
        let std_lo = norm(m, (m.mean - m.std_dev).max(0.0));
        let std_hi = norm(m, m.mean + m.std_dev).min(y_max);
        chart.draw_series(std::iter::once(Rectangle::new(
            [(x_l, std_lo), (x_r, std_hi)],
            RGBColor(0x1f, 0x77, 0xb4).mix(0.55).filled(),
        )))?;

        // Cienka linia mean
        let mean_n = norm(m, m.mean).min(y_max);
        chart.draw_series(std::iter::once(Rectangle::new(
            [(x_l, (mean_n - 0.012).max(0.0)), (x_r, mean_n + 0.012)],
            RGBColor(0x1f, 0x77, 0xb4).filled(),
        )))?;

        // Adnotacja pass_rate
        chart.draw_series(std::iter::once(Text::new(
            format!("{:.0}%", m.pass_rate * 100.0),
            (i as f64 + 0.4, (mean_n + 0.10).min(y_max - 0.05)),
            ("sans-serif", 12).into_font(),
        )))?;
    }

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.85))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}
