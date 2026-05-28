use crate::{
    controller::Controller,
    lqr::{
        care::{CareError, RiccatiSolution, SolverParams, build_q_diagonal, build_r_diagonal, solve_care},
        linearize::{LinearizedModel, linearize, state_to_vec, vec_to_input},
    },
    target::FlightTarget,
};
use drone_model::{
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};
use nalgebra::{DMatrix, DVector};

pub struct LqrController {
    k: DMatrix<f64>,
    x0: DVector<f64>,
    u0: DVector<f64>,
    input_template: KnownActuatorInput,
    u_limits: Vec<(f64, f64)>,
}

impl LqrController {
    /// Low-level constructor from a pre-computed Riccati solution.
    /// Prefer `design()` for typical use. `pub(crate)` to avoid leaking
    /// the `RiccatiSolution` implementation detail as a public contract.
    pub(crate) fn new(
        solution: RiccatiSolution,
        linearized: &LinearizedModel,
        input_template: KnownActuatorInput,
        u_limits: Vec<(f64, f64)>,
    ) -> Self {
        Self {
            k: solution.k,
            x0: linearized.x0.clone(),
            u0: linearized.u0.clone(),
            input_template,
            u_limits,
        }
    }

    /// Design an LQR controller around the given trim state.
    ///
    /// Returns `Err(CareError)` if the CARE solver fails, e.g. because the
    /// system is uncontrollable or the weight matrices have wrong dimensions.
    pub fn design(
        model: &dyn VehicleModel,
        trim_state: &DroneState,
        q_weights: &[f64],
        r_weights: &[f64],
        u_limits: Vec<(f64, f64)>,
    ) -> Result<Self, CareError> {
        let trim_input = model.equilibrium_input();
        let linearized = linearize(model, trim_state, &trim_input);

        let q = build_q_diagonal(q_weights);
        let r = build_r_diagonal(r_weights);

        let params = SolverParams::default();
        let solution = solve_care(&linearized.a, &linearized.b, &q, &r, &params)?;

        // Diagnostics available via solution.flow_steps / solution.newton_iters.
        Ok(Self::new(solution, &linearized, trim_input, u_limits))
    }

    pub fn compute_control(&self, state: &DroneState) -> DVector<f64> {
        let x = state_to_vec(state);
        let dx = x - &self.x0;
        let du = &self.k * &dx;
        let u_raw = &self.u0 - du;

        let m = u_raw.len();
        let mut u = u_raw;
        for i in 0..m.min(self.u_limits.len()) {
            let (lo, hi) = self.u_limits[i];
            u[i] = u[i].clamp(lo, hi);
        }
        u
    }
}

impl Controller for LqrController {
    /// Note: `_target` and `_dt` are intentionally ignored.
    ///
    /// LQR is designed offline for a fixed operating point (trim state).  It
    /// stabilises around that point regardless of the runtime target.  If you
    /// need the controller to *track* arbitrary targets use `LqiController`,
    /// which integrates the tracking error at runtime.
    fn update(
        &mut self,
        state: &DroneState,
        _target: &FlightTarget,
        _dt: TimeStep,
    ) -> KnownActuatorInput {
        let u = self.compute_control(state);
        vec_to_input(&u, &self.input_template)
    }

    fn reset(&mut self) {
        //
    }

    fn name(&self) -> &str {
        "LQR"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lqr::linearize::input_to_vec;
    use drone_model::{state::DroneState, vehicle::quadrotor::QuadrotorModel};
    use nalgebra::{UnitQuaternion, Vector3};

    fn hover_state() -> DroneState {
        DroneState {
            position: Vector3::new(0.0, 0.0, 10.0),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            actuator_state: None,
        }
    }

    #[test]
    fn lqr_designs_for_quadrotor() {
        let model = QuadrotorModel::mini3_simple();
        let hover = hover_state();

        // Weights: position, velocity, quaternion, ω
        let q_weights = vec![
            10.0, 10.0, 100.0, // xyz — altitude more important
            1.0, 1.0, 1.0, // vxyz
            5.0, 5.0, 5.0, // ω
            50.0, 50.0, 50.0, 50.0, // quaternion
        ];
        let r_weights = vec![0.1, 0.1, 0.1, 0.1]; // 4 engines

        // Limits: engine speeds [0, max]
        let hover_w = match model.equilibrium_input() {
            drone_model::vehicle::KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };
        let u_limits = vec![(0.0, hover_w * 2.0); 4];

        let result = LqrController::design(&model, &hover, &q_weights, &r_weights, u_limits);

        assert!(result.is_ok(), "LQR design failed: {:?}", result.err());
        let ctrl = result.unwrap();
        assert!(ctrl.k.norm() > 0.01, "K must be non-zero");
    }

    #[test]
    fn lqr_hover_gives_close_equilibrium() {
        let model = QuadrotorModel::mini3_simple();
        let hover = hover_state();

        let q_weights = vec![10.0; 13];
        let r_weights = vec![0.1; 4];
        let hover_w = match model.equilibrium_input() {
            drone_model::vehicle::KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };
        let u_limits = vec![(0.0, hover_w * 2.0); 4];

        let mut ctrl =
            LqrController::design(&model, &hover, &q_weights, &r_weights, u_limits).unwrap();

        // At state = trim point → control ≈ u₀
        let target = FlightTarget::altitude(10.0);
        let input = ctrl.update(&hover, &target, TimeStep::constant(0.005));

        let u_vec = input_to_vec(&input);
        let u0_norm = ctrl.u0.norm();
        let u_norm = u_vec.norm();

        assert!(
            (u_norm - u0_norm).abs() / u0_norm < 0.05,
            "At working point control ≈ u₀: u={:.2}, u₀={:.2}",
            u_norm,
            u0_norm
        );
    }
}
