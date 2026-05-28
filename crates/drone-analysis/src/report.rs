use crate::align::AlignedTrajectory;

/// Position error threshold used to determine "model valid until t":
/// the last time at which the 3-D position error is below this value.
/// Exposed as a public constant so `ValidateConfig` and callers can
/// reference it without a magic number.
pub const VALID_POSITION_THRESHOLD_M: f64 = 2.0;

pub struct ValidationReport {
    pub source_file: String,
    pub flight_duration_s: f64,
    pub n_points: usize,

    pub position_rms_m: f64,
    pub position_max_m: f64,
    pub velocity_rms_ms: f64,

    pub valid_until_s: f64,

    pub trajectory: AlignedTrajectory,
}

impl ValidationReport {
    pub fn from_aligned(
        aligned: AlignedTrajectory,
        source_file: String,
        valid_threshold_m: f64,
    ) -> Self {
        let n = aligned.points.len();
        let pos_rms = aligned.position_rms();
        let pos_max = aligned.position_max();
        let vel_rms = aligned.velocity_rms();
        let dur = aligned.duration_s;

        let valid_until = aligned
            .points
            .iter()
            .take_while(|p| p.pos_error < valid_threshold_m)
            .last()
            .map(|p| p.time)
            .unwrap_or(0.0);

        Self {
            source_file,
            flight_duration_s: dur,
            n_points: n,
            position_rms_m: pos_rms,
            position_max_m: pos_max,
            velocity_rms_ms: vel_rms,
            valid_until_s: valid_until,
            trajectory: aligned,
        }
    }

    pub fn print(&self) {
        println!("\n╔══════════════════════════════════════════════════════════╗");
        println!("║  Model validation: {}", self.source_file);
        println!("╠══════════════════════════════════════════════════════════╣");
        println!(
            "║  Flight time:          {:.1}s ({} points)",
            self.flight_duration_s, self.n_points
        );
        println!("║  RMS position error:  {:.3}m", self.position_rms_m);
        println!("║  Max position error:   {:.3}m", self.position_max_m);
        println!("║  RMS velocity error:  {:.3}m/s", self.velocity_rms_ms);
        println!("║  Model good until:    {:.1}s", self.valid_until_s);
        println!("╠══════════════════════════════════════════════════════════╣");

        let quality = if self.position_rms_m < 0.5 {
            "Excellent (< 0.5m RMS)"
        } else if self.position_rms_m < 2.0 {
            "Good (< 2m RMS)"
        } else if self.position_rms_m < 5.0 {
            "Approximate (< 5m RMS)"
        } else {
            "Poor (> 5m RMS) — check model parameters"
        };
        println!("║  Rating: {}", quality);
        println!("╚══════════════════════════════════════════════════════════╝");
    }

    pub fn to_csv(&self) -> String {
        self.trajectory.to_csv()
    }
}
