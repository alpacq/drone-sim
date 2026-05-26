#[derive(Debug, Clone)]
pub struct F16Params {
    pub mass: f64,
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
}

impl F16Params {
    pub fn f16a() -> Self {
        Self {
            mass: 9_295.44,

            ixx: 12_874.8,
            iyy: 75_673.6,
            izz: 85_552.1,
            ixy: 0.0,
            ixz: 1_331.4,
            iyz: 0.0,
        }
    }
}
