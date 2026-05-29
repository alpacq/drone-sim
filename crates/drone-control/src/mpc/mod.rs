//! Model Predictive Controller.
//!
//! Uses a condensed QP formulation over a finite horizon, solved by
//! projected gradient descent.  Re-linearises the model at every control step
//! (receding horizon).
//!
//! # Limitations
//!
//! * The controller stores an `Arc<dyn VehicleModel>`, so it cannot be created
//!   through the generic `ControllerFactory` signature (which only provides
//!   `&dyn VehicleModel`).  Use [`MpcController::new`] or
//!   [`MpcController::for_quadrotor`] directly instead.

use crate::{
    controller::Controller,
    lqr::linearize::{discretize_euler, input_to_vec, linearize, state_to_vec, vec_to_input},
    target::FlightTarget,
};
use drone_model::{
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

/// Model Predictive Controller.
///
/// Solves a condensed finite-horizon QP at every control step by projected
/// gradient descent.  The prediction and control horizons are both `horizon`
/// steps.
///
/// # Cost function (per horizon)
///
/// ```text
/// J = Σ_{k=1}^{N} (xk − xref)ᵀ Q (xk − xref)  +  Σ_{k=0}^{N-1} δukᵀ R δuk
/// ```
///
/// where `δu = u − u_eq`, `Q = diag(q_weights)`, `R = diag(r_weights)`.
pub struct MpcController {
    /// Vehicle model used for linearisation.
    model: Arc<dyn VehicleModel>,
    /// Prediction (= control) horizon in steps.
    pub horizon: usize,
    /// Prediction step size [s].
    pub dt: f64,
    /// State cost weights — must have exactly 13 elements
    /// (pos xyz, vel xyz, ω xyz, q ijkw).
    pub q_weights: Vec<f64>,
    /// Control cost weights — must have exactly `m` elements (4 for quadrotor).
    pub r_weights: Vec<f64>,
    /// Per-actuator bounds: `u_limits[i] = (lo, hi)`.
    pub u_limits: Vec<(f64, f64)>,
    /// Maximum projected-gradient iterations per solve.
    pub max_iter: usize,
    /// Warm-start: previous optimal control sequence (length N·m), if any.
    prev_u: Option<DVector<f64>>,
}

impl MpcController {
    /// Create a new MPC controller.
    ///
    /// * `q_weights` — 13 entries (state cost diagonal).
    /// * `r_weights` — `actuator_count` entries (control cost diagonal).
    /// * `u_limits` — `actuator_count` entries of `(lo, hi)` bounds.
    pub fn new(
        model: Arc<dyn VehicleModel>,
        horizon: usize,
        dt: f64,
        q_weights: Vec<f64>,
        r_weights: Vec<f64>,
        u_limits: Vec<(f64, f64)>,
    ) -> Self {
        assert!(horizon >= 1, "horizon must be >= 1");
        assert_eq!(q_weights.len(), 13, "q_weights must have 13 entries");
        let m = model.actuator_count();
        assert_eq!(
            r_weights.len(),
            m,
            "r_weights must have {m} entries (actuator_count)"
        );
        assert_eq!(
            u_limits.len(),
            m,
            "u_limits must have {m} entries (actuator_count)"
        );
        Self {
            model,
            horizon,
            dt,
            q_weights,
            r_weights,
            u_limits,
            max_iter: 150,
            prev_u: None,
        }
    }

    /// Create an MPC controller for a quadrotor with sensible defaults.
    ///
    /// The upper motor-speed bound is set to twice the hover speed.
    pub fn for_quadrotor(model: Arc<dyn VehicleModel>, horizon: usize, dt: f64) -> Self {
        let hover_w = match model.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!("for_quadrotor requires a quadrotor model"),
        };
        let q_weights = vec![
            10.0, 10.0, 50.0, // x y z
            1.0, 1.0, 5.0, // vx vy vz
            2.0, 2.0, 2.0, // ωx ωy ωz
            5.0, 5.0, 5.0, 5.0, // qi qj qk qw
        ];
        let r_weights = vec![0.1; 4];
        let u_limits = vec![(0.0, hover_w * 2.0); 4];
        Self::new(model, horizon, dt, q_weights, r_weights, u_limits)
    }

    /// Construct a reference state vector from a [`FlightTarget`], using
    /// the current state for axes that are not controlled.
    fn build_ref(target: &FlightTarget, current: &DroneState) -> DVector<f64> {
        let mut r = state_to_vec(current);
        if let Some(x) = target.x {
            r[0] = x;
        }
        if let Some(y) = target.y {
            r[1] = y;
        }
        if let Some(z) = target.z {
            r[2] = z;
        }
        // Drive velocities and angular rates to zero.
        for i in 3..9 {
            r[i] = 0.0;
        }
        // Drive orientation toward identity quaternion [i=0, j=0, k=0, w=1].
        r[9] = 0.0; // q.i
        r[10] = 0.0; // q.j
        r[11] = 0.0; // q.k
        r[12] = 1.0; // q.w
        r
    }

    /// Build the condensed Φ and Γ matrices for the horizon.
    ///
    /// * `Φ` (N·n × n): maps initial state to predicted state sequence.
    /// * `Γ` (N·n × N·m): maps control sequence to predicted state sequence.
    ///
    /// Recurrence: `X = Φ·x0 + Γ·U` where `X = [x1; …; xN]`, `U = [u0; …; u_{N-1}]`.
    fn build_phi_gamma(
        ad: &DMatrix<f64>,
        bd: &DMatrix<f64>,
        n_horizon: usize,
    ) -> (DMatrix<f64>, DMatrix<f64>) {
        let n = ad.nrows();
        let m = bd.ncols();
        let mut phi = DMatrix::zeros(n_horizon * n, n);
        let mut gamma = DMatrix::zeros(n_horizon * n, n_horizon * m);

        // ad_pow = Ad^{k+1} after iteration k
        let mut ad_pow = ad.clone();
        for k in 0..n_horizon {
            // Φ block row k: Ad^{k+1}
            phi.view_mut((k * n, 0), (n, n)).copy_from(&ad_pow);

            // Γ block (k, j) = Ad^{k-j} · Bd  for j in [0, k]
            let mut ad_pow2 = DMatrix::<f64>::identity(n, n);
            for j in 0..=k {
                let g = &ad_pow2 * bd;
                gamma
                    .view_mut((k * n, (k - j) * m), (n, m))
                    .copy_from(&g);
                ad_pow2 = &ad_pow2 * ad;
            }

            ad_pow = &ad_pow * ad;
        }
        (phi, gamma)
    }

    /// Build the QP matrices from Φ, Γ, and the weight diagonals.
    ///
    /// The QP is formulated in terms of `δU = U − U_eq` so that the control
    /// cost penalises deviations from equilibrium, not from zero.
    ///
    /// * `H = Γᵀ Q̄ Γ + R̄`  (symmetric, N·m × N·m)
    /// * `f = 2 Γᵀ Q̄ (Φ x0 + Γ U_eq − X_ref)`  (N·m × 1)
    ///
    /// Cost: `J(δU) = δUᵀ H δU + fᵀ δU + const`
    fn build_qp(
        phi: &DMatrix<f64>,
        gamma: &DMatrix<f64>,
        q_bar: &DMatrix<f64>,
        r_bar: &DMatrix<f64>,
        x0: &DVector<f64>,
        x_ref: &DVector<f64>,
        u_eq: &DVector<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        let n_state = x_ref.len();
        let n_full = phi.nrows();
        let x_ref_full = DVector::from_fn(n_full, |i, _| x_ref[i % n_state]);
        // Prediction error at δU = 0 (i.e. applying equilibrium control).
        let e0 = phi * x0 + gamma * u_eq - x_ref_full;

        let gt_qbar = gamma.transpose() * q_bar;
        let h = &gt_qbar * gamma + r_bar;
        let f = 2.0 * &gt_qbar * &e0;
        (h, f)
    }

    /// Projected gradient descent to minimise `J(U) = Uᵀ H U + fᵀ U`
    /// subject to element-wise bounds.
    ///
    /// Step size: `α = 1 / (2·‖H‖_F + ε)` — conservative Lipschitz bound.
    fn projected_gradient(
        h: &DMatrix<f64>,
        f: &DVector<f64>,
        u_lo: &DVector<f64>,
        u_hi: &DVector<f64>,
        u_init: &DVector<f64>,
        max_iter: usize,
    ) -> DVector<f64> {
        let alpha = 1.0 / (2.0 * h.norm() + 1e-8);
        let mut u = u_init.clone();
        for _ in 0..max_iter {
            let grad = 2.0 * (h * &u) + f;
            u -= alpha * &grad;
            // Project onto per-element bounds.
            for i in 0..u.len() {
                u[i] = u[i].clamp(u_lo[i], u_hi[i]);
            }
        }
        u
    }
}

impl Controller for MpcController {
    fn update(
        &mut self,
        state: &DroneState,
        target: &FlightTarget,
        dt: TimeStep,
    ) -> KnownActuatorInput {
        let m = self.u_limits.len();
        let n_steps = self.horizon;

        // 1. Linearise and discretise at the current operating point.
        let trim_input = self.model.equilibrium_input();
        let lm = linearize(self.model.as_ref(), state, &trim_input);
        let (ad, bd) = discretize_euler(&lm.a, &lm.b, dt.seconds());

        // 2. Build condensed prediction matrices.
        let (phi, gamma) = Self::build_phi_gamma(&ad, &bd, n_steps);

        // 3. Block-diagonal weight matrices.
        let n_full = n_steps * 13;
        let nm = n_steps * m;
        let mut q_bar = DMatrix::zeros(n_full, n_full);
        for k in 0..n_steps {
            for i in 0..13 {
                q_bar[(k * 13 + i, k * 13 + i)] = self.q_weights[i];
            }
        }
        let mut r_bar = DMatrix::zeros(nm, nm);
        for k in 0..n_steps {
            for i in 0..m {
                r_bar[(k * m + i, k * m + i)] = self.r_weights[i];
            }
        }

        // 4. Build QP in δU coordinates (deviation from equilibrium).
        let x0 = state_to_vec(state);
        let x_ref = Self::build_ref(target, state);
        let u0_vec = input_to_vec(&trim_input);
        // U_eq repeated over the horizon.
        let mut u_eq_full = DVector::zeros(nm);
        for k in 0..n_steps {
            u_eq_full.rows_mut(k * m, m).copy_from(&u0_vec);
        }
        let (h, f) =
            Self::build_qp(&phi, &gamma, &q_bar, &r_bar, &x0, &x_ref, &u_eq_full);

        // 5. Element-wise bounds for δU = U − U_eq.
        let mut du_lo = DVector::zeros(nm);
        let mut du_hi = DVector::zeros(nm);
        for k in 0..n_steps {
            for i in 0..m {
                du_lo[k * m + i] = self.u_limits[i].0 - u0_vec[i];
                du_hi[k * m + i] = self.u_limits[i].1 - u0_vec[i];
            }
        }

        // 6. Warm start (in δU coordinates).
        let du_init = match &self.prev_u {
            Some(prev) if prev.len() == nm => {
                // Shift by one step: drop first m elements, pad zeros at end.
                let mut shifted = DVector::zeros(nm);
                shifted.rows_mut(0, nm - m).copy_from(&prev.rows(m, nm - m));
                shifted
            }
            _ => DVector::zeros(nm), // δU = 0 means apply equilibrium
        };

        // 7. Solve for optimal δU.
        let du_opt =
            Self::projected_gradient(&h, &f, &du_lo, &du_hi, &du_init, self.max_iter);
        self.prev_u = Some(du_opt.clone());

        // 8. Apply first control step: u = u_eq + δu.
        let u_first = du_opt.rows(0, m) + u0_vec;
        vec_to_input(&u_first, &trim_input)
    }

    fn reset(&mut self) {
        self.prev_u = None;
    }

    fn name(&self) -> &str {
        "MPC"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::vehicle::quadrotor::QuadrotorModel;
    use nalgebra::{UnitQuaternion, Vector3};

    fn hover_state(z: f64) -> DroneState {
        DroneState {
            position: Vector3::new(0.0, 0.0, z),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    #[test]
    fn mpc_at_trim_gives_equilibrium_control() {
        let model = Arc::new(QuadrotorModel::mini3_simple());
        let mut mpc = MpcController::for_quadrotor(model.clone(), 5, 0.02);

        let state = hover_state(5.0);
        let target = FlightTarget::altitude(5.0);
        let dt = TimeStep::constant(0.02);
        let input = mpc.update(&state, &target, dt);

        // At trim the MPC should return ≈ equilibrium input.
        let u = input_to_vec(&input);
        let u0 = input_to_vec(&model.equilibrium_input());
        let err = (&u - &u0).norm() / u0.norm();
        assert!(
            err < 0.2,
            "At trim, MPC should return ≈ equilibrium: err={err:.3}, u={:?}, u0={:?}",
            u.as_slice(),
            u0.as_slice()
        );
    }

    #[test]
    fn mpc_above_target_reduces_throttle() {
        let model = Arc::new(QuadrotorModel::mini3_simple());
        let mut mpc = MpcController::for_quadrotor(model.clone(), 5, 0.02);

        // Drone is 5 m above the target.
        let state = hover_state(10.0);
        let target = FlightTarget::altitude(5.0);
        let dt = TimeStep::constant(0.02);
        let input = mpc.update(&state, &target, dt);

        let u = input_to_vec(&input);
        let u0 = input_to_vec(&model.equilibrium_input());
        let avg = u.mean();
        let avg0 = u0.mean();
        assert!(
            avg < avg0,
            "Above target: avg motor speed {avg:.1} should be less than hover {avg0:.1}"
        );
    }

    #[test]
    fn mpc_reset_clears_warm_start() {
        let model = Arc::new(QuadrotorModel::mini3_simple());
        let mut mpc = MpcController::for_quadrotor(model.clone(), 5, 0.02);

        let state = hover_state(5.0);
        let target = FlightTarget::altitude(5.0);
        let dt = TimeStep::constant(0.02);
        mpc.update(&state, &target, dt);

        assert!(mpc.prev_u.is_some());
        mpc.reset();
        assert!(mpc.prev_u.is_none());
    }

    #[test]
    fn build_phi_gamma_dimensions() {
        // Check that condensed matrices have correct dimensions.
        let n = 4;
        let m = 2;
        let horizon = 3;
        let ad = DMatrix::<f64>::identity(n, n);
        let bd = DMatrix::<f64>::zeros(n, m);
        let (phi, gamma) = MpcController::build_phi_gamma(&ad, &bd, horizon);
        assert_eq!(phi.nrows(), horizon * n);
        assert_eq!(phi.ncols(), n);
        assert_eq!(gamma.nrows(), horizon * n);
        assert_eq!(gamma.ncols(), horizon * m);
    }

    #[test]
    fn projected_gradient_unconstrained_min() {
        // Minimise J = u^2 - 2u  =>  H = [[1]], f = [[-2]], optimal u = 1
        let h = DMatrix::from_element(1, 1, 1.0);
        let f = DVector::from_element(1, -2.0);
        let lo = DVector::from_element(1, -100.0);
        let hi = DVector::from_element(1, 100.0);
        let init = DVector::from_element(1, 0.0);
        let u = MpcController::projected_gradient(&h, &f, &lo, &hi, &init, 500);
        assert!(
            (u[0] - 1.0).abs() < 0.05,
            "optimal u should ≈ 1, got {:.4}",
            u[0]
        );
    }
}
