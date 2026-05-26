use super::aero_tables::*;
use crate::math::atmosphere::AtmosphereModel;
use crate::state::DroneState;
use crate::vehicle::{ForcesAndMoments, KnownActuatorInput};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct AeroState {
    /// Airspeed [m/s]
    pub airspeed: f64,
    /// Angle of attack α [rad]
    pub alpha_rad: f64,
    /// Angle of side slip β [rad]
    pub beta_rad: f64,
    /// Mach number [-]
    pub mach: f64,
    /// Dynamic pressure q̄ = ½ρV² [Pa]
    pub qbar: f64,
    /// Speed in body frame [u, v, w] [m/s] (ENU)
    pub v_body: Vector3<f64>,
}

impl AeroState {
    pub fn compute(state: &DroneState, atmosphere: &dyn AtmosphereModel) -> Self {
        let altitude = state.position.z.max(0.0);

        let v_body = state.orientation.inverse() * state.velocity;

        let u = v_body.x;
        let v = v_body.y;
        let w = v_body.z;

        let airspeed = (u * u + v * v + w * w).sqrt().max(1.0);

        let alpha_rad = (-w / airspeed.max(1e-3)).asin().clamp(-0.5, 0.785);

        let beta_rad = (v / airspeed).asin().clamp(-0.524, 0.524);

        let mach = atmosphere.mach(altitude, airspeed);
        let qbar = atmosphere.dynamic_pressure(altitude, airspeed);

        Self {
            airspeed,
            alpha_rad,
            beta_rad,
            mach,
            qbar,
            v_body,
        }
    }

    pub fn alpha_deg(&self) -> f64 {
        self.alpha_rad.to_degrees()
    }

    pub fn beta_deg(&self) -> f64 {
        self.beta_rad.to_degrees()
    }
}

#[derive(Debug, Clone)]
pub struct F16GeomParams {
    pub wing_area: f64,  // [m^2]
    pub wingspan: f64,   // [m]
    pub mean_chord: f64, // [m]
}

impl F16GeomParams {
    pub fn f16a() -> Self {
        Self {
            wing_area: 27.87,
            wingspan: 9.45,
            mean_chord: 3.45,
        }
    }
}

pub fn compute_aero(
    aero: &AeroState,
    angular: &Vector3<f64>,
    input: &KnownActuatorInput,
    geom: &F16GeomParams,
    thrust_n: f64,
) -> ForcesAndMoments {
    let (de_deg, da_deg, dr_deg) = extract_controls(input);

    let alpha = aero.alpha_deg();
    let beta = aero.beta_deg();
    let v = aero.airspeed;
    let qbar = aero.qbar;
    let s = geom.wing_area;
    let b = geom.wingspan;
    let cbar = geom.mean_chord;

    let p = angular.x;
    let q = angular.y;
    let r = angular.z;

    let pb2v = p * b / (2.0 * v);
    let qc2v = q * cbar / (2.0 * v);
    let rb2v = r * b / (2.0 * v);

    let cx = interp1d(ALPHA_BREAK, CX_ALPHA, alpha);
    let cz = interp1d(ALPHA_BREAK, CZ_ALPHA, alpha);
    let cm_base = interp1d(ALPHA_BREAK, CM_ALPHA, alpha);
    let cy_base = interp1d(BETA_BREAK, CY_BETA, beta);
    let cl_base = interp1d(BETA_BREAK, CL_BETA, beta);
    let cn_base = interp1d(BETA_BREAK, CN_BETA, beta);

    let cxq = interp1d(ALPHA_BREAK, CXQ_ALPHA, alpha);
    let czq = interp1d(ALPHA_BREAK, CZQ_ALPHA, alpha);
    let cmq = interp1d(ALPHA_BREAK, CMQ_ALPHA, alpha);
    let clp = interp1d(ALPHA_BREAK, CLP_ALPHA, alpha);
    let cnr = interp1d(ALPHA_BREAK, CNR_ALPHA, alpha);
    let clr = interp1d(ALPHA_BREAK, CLR_ALPHA, alpha);
    let cnp = interp1d(ALPHA_BREAK, CNP_ALPHA, alpha);

    let cz_de = CZ_DE_PER_DEG * de_deg;
    let cm_de = CM_DE_PER_DEG * de_deg;

    let cl_da_eff = interp1d(ALPHA_BREAK, CL_DA_ALPHA, alpha);
    let cn_da_eff = interp1d(ALPHA_BREAK, CN_DA_ALPHA, alpha);
    let cl_da = cl_da_eff * da_deg;
    let cn_da = cn_da_eff * da_deg;

    let cl_dr = CL_DR_PER_DEG * dr_deg;
    let cn_dr = CN_DR_PER_DEG * dr_deg;

    let cx_total = cx + cxq * qc2v;
    let cz_total = cz + cz_de + czq * qc2v;
    let cm_total = cm_base + cm_de + cmq * qc2v;
    let cy_total = cy_base;
    let cl_total = cl_base + cl_da + cl_dr + clp * pb2v + clr * rb2v;
    let cn_total = cn_base + cn_da + cn_dr + cnp * pb2v + cnr * rb2v;

    let alpha_rad = aero.alpha_rad;
    let fx_body = cx_total * qbar * s + thrust_n * alpha_rad.cos();
    let fy_body = cy_total * qbar * s;
    let fz_body = -cz_total * qbar * s + thrust_n * alpha_rad.sin();

    let l_moment = cl_total * qbar * s * b; // roll
    let m_moment = cm_total * qbar * s * cbar; // pitch
    let n_moment = cn_total * qbar * s * b; // yaw

    ForcesAndMoments {
        force: Vector3::new(fx_body, fy_body, fz_body),
        torque: Vector3::new(l_moment, m_moment, n_moment),
    }
}

fn extract_controls(input: &KnownActuatorInput) -> (f64, f64, f64) {
    match input {
        KnownActuatorInput::FixedWing {
            aileron,
            elevator,
            rudder,
            ..
        } => {
            let de = elevator * 25.0;
            let da = aileron * 21.5;
            let dr = rudder * 30.0;
            (de, da, dr)
        }
        _ => panic!("F16Aero: expected FixedWing input"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::atmosphere::ConstantDensity;
    use crate::state::DroneState;
    use nalgebra::{UnitQuaternion, Vector3};

    fn cruise_state() -> DroneState {
        // F-16 in vertical flight V = 192 m/s, α ≈ 2°
        let alpha_rad = 2_f64.to_radians();
        DroneState {
            position: Vector3::new(0.0, 0.0, 5000.0),
            // Velocity in world frame — vertical flight
            velocity: Vector3::new(192.0 * alpha_rad.cos(), 0.0, 192.0 * alpha_rad.sin()),
            orientation: UnitQuaternion::from_euler_angles(0.0, -alpha_rad, 0.0),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    fn neutral_input() -> KnownActuatorInput {
        KnownActuatorInput::FixedWing {
            throttle: 0.5,
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
        }
    }

    #[test]
    fn aero_state_correct_for_vertical_flight() {
        let atmo = ConstantDensity::sea_level();
        let state = cruise_state();
        let aero = AeroState::compute(&state, &atmo);

        // Airspeed ≈ 192 m/s
        assert!(
            (aero.airspeed - 192.0).abs() < 2.0,
            "V = {:.1}",
            aero.airspeed
        );

        // beta angle = 0 (symmetric flight)
        assert!(aero.beta_rad.abs() < 0.01, "β = {:.4} rad", aero.beta_rad);
    }

    #[test]
    fn lift_correct_for_vertical_flight() {
        let atmo = ConstantDensity::sea_level();
        let state = cruise_state();
        let aero = AeroState::compute(&state, &atmo);
        let geom = F16GeomParams::f16a();

        let fm = compute_aero(
            &aero,
            &Vector3::zeros(),
            &neutral_input(),
            &geom,
            0.0, // no thrust — check only aerodynamics
        );

        // α > 0 should have positive lift (Fz > 0 in ENU)
        assert!(
            fm.force.z > 0.0,
            "Fz should be positive (lift): {:.1} N",
            fm.force.z
        );
    }

    #[test]
    fn elevator_changes_pitch_moment() {
        let atmo = ConstantDensity::sea_level();
        let state = cruise_state();
        let aero = AeroState::compute(&state, &atmo);
        let geom = F16GeomParams::f16a();

        let fm_neutral = compute_aero(&aero, &Vector3::zeros(), &neutral_input(), &geom, 0.0);

        let up_input = KnownActuatorInput::FixedWing {
            throttle: 0.5,
            aileron: 0.0,
            elevator: 0.5, // nose up
            rudder: 0.0,
        };
        let fm_up = compute_aero(&aero, &Vector3::zeros(), &up_input, &geom, 0.0);

        // Elevator should increase pitch moment
        assert_ne!(
            fm_neutral.torque.y, fm_up.torque.y,
            "Elevator should increase pitch moment"
        );
    }

    #[test]
    fn rudders_generate_roll_moment() {
        let atmo = ConstantDensity::sea_level();
        let state = cruise_state();
        let aero = AeroState::compute(&state, &atmo);
        let geom = F16GeomParams::f16a();

        let roll_input = KnownActuatorInput::FixedWing {
            throttle: 0.5,
            aileron: 0.5,
            elevator: 0.0,
            rudder: 0.0,
        };
        let fm = compute_aero(&aero, &Vector3::zeros(), &roll_input, &geom, 0.0);

        assert!(
            fm.torque.x.abs() > 0.1,
            "Rudders should generate roll moment: {:.2} N·m",
            fm.torque.x
        );
    }
}
