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
    pub fn mini3() -> Self {
        Self::new(0.249, 0.085, 1.5e-7, 2.5e-9, 1.43e-4, 1.43e-4, 2.89e-4)
    }
}
