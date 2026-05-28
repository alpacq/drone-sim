use chrono::{DateTime, Utc};

#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    pub index: u32,
    pub timestamp: Option<DateTime<Utc>>,
    pub duration_ms: u32,

    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub rel_alt: Option<f32>,
    pub abs_alt: Option<f32>,

    pub gimbal_yaw: Option<f32>,
    pub gimbal_pitch: Option<f32>,
    pub gimbal_roll: Option<f32>,

    pub iso: Option<u32>,
    pub shutter: Option<String>,
    pub fnum: Option<u32>,
    pub color_temp: Option<u32>,
}

impl TelemetryFrame {
    pub fn dt_seconds(&self) -> f64 {
        self.duration_ms as f64 / 1000.0
    }

    pub fn has_gps(&self) -> bool {
        self.latitude.is_some() && self.longitude.is_some()
    }
}
