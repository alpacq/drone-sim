use nalgebra::{Matrix3, Vector3};
use serde::Deserialize;

/// Physical parameters of the drone - constant for given model
/// Loaded from TOML file
#[derive(Debug, Clone, Deserialize)]
pub struct DroneParams {
    /// full mass [kg]. Mini 3: 0.249
    pub mass: f64,

    /// arm length: from mass center to engine [m]
    pub arm_length: f64,

    /// thrust coefficient: F = k_thrust * ω²  [N·s²/rad²]
    pub k_thrust: f64,

    /// torque coefficient: τ = k_torque * ω²  [N·m·s²/rad²]
    pub k_torque: f64,

    /// inertia tensor [kg·m²]
    /// for symmetric quadrotor it is diagonal matrix
    /// [[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]]
    #[serde(skip)]
    pub inertia: Matrix3<f64>,

    /// inversion of inertia tensor - computed once, used often
    #[serde(skip)]
    pub inertia_inv: Matrix3<f64>,
}

impl DroneParams {
    /// constructor computing 'inertia_inv'
    pub fn new(
        mass: f64,
        arm_length: f64,
        k_thrust: f64,
        k_torque: f64,
        ixx: f64,
        iyy: f64,
        izz: f64,
    ) -> Self {
        let inertia = Matrix3::from_diagonal(&Vector3::new(ixx, iyy, izz));
        let inertia_inv = inertia
            .try_inverse()
            .expect("Inertia tensor must be invertable");

        Self {
            mass,
            arm_length,
            k_thrust,
            k_torque,
            inertia,
            inertia_inv,
        }
    }

    /// parameters comparable to DJI Mini 3
    /// One engine mass ≈ 0.020 kg
    /// Central mass ≈ 0.249 - 4×0.020 = 0.169 kg
    /// Arm length = 0.085 m
    /// Ixx_engines = 4 × m_engine × r² = 4 × 0.020 × 0.060² = 2.88e-4 kg·m²
    /// Ixx_central ≈ (1/12) × m_central × d² (d ≈ 0.06m body width)
    ///             = (1/12) × 0.169 × 0.06² = 5.1e-5 kg·m²
    /// Ixx ≈ 2.88e-4 + 5.1e-5 ≈ 3.4e-4 kg·m²
    /// Iyy = Ixx
    /// Izz ≈ 3.4e-4 + 3.4e-4 = 6.8e-4 kg·m²
    /// P = 3.71 W (rated), 22.3 W (max)
    /// τ = P / ω
    /// At hover (ω = 632.5 rad/s, P ≈ 3.71 W):
    /// τ_hover = 3.71 / 632.5 ≈ 0.00587 N·m
    /// k_torque = τ / ω² = 0.00587 / 632.5² = 1.466e-8 N·m·s²/rad²
    /// At max RPM (ω = 10700 rpm = 1120 rad/s, P = 22.3 W):
    /// τ_max = 22.3 / 1120 ≈ 0.0199 N·m
    /// k_torque = 0.0199 / 1120² = 1.586e-8
    pub fn mini3() -> Self {
        Self::new(
            0.249,    // mass [kg]
            0.085,    // arm [m] — approximation, Mini 3 has H-frame geometry, not X-frame
            1.526e-6, // k_thrust [N·s²/rad²]
            1.5e-8,   // k_torque [N·m·s²/rad²]
            3.4e-4,   // Ixx [kg·m²]
            3.4e-4,   // Iyy [kg·m²]
            6.8e-4,   // Izz [kg·m²]
        )
    }
}
