//! Model Predictive Controller.
//!
//! Uses a condensed QP formulation over a finite horizon, solved by
//! projected gradient descent.  Re-linearises the model at every control step
//! (receding horizon).
//!
//! # Integral augmentation
//!
//! The MPC accumulates position error integrals `ξ = [ξ_x, ξ_y, ξ_z]` at
//! runtime.  These act as a bias on the QP cost gradient, pushing the
//! optimiser to correct persistent offsets caused by unmodeled dynamics
//! (e.g. motor lag).  This is the standard "offset-free MPC" approach.
//!
//! # Limitations
//!
//! * The controller stores an `Arc<dyn VehicleModel>`, so it cannot be created
//!   through the generic `ControllerFactory` signature (which only provides
//!   `&dyn VehicleModel`).  Use [`MpcController::new`] or
//!   [`MpcController::for_quadrotor`] directly instead.

use crate::{
    controller::Controller,
    lqr::linearize::{discretize_implicit_euler, input_to_vec, linearize, state_to_vec, vec_to_input},
    target::FlightTarget,
};
use drone_model::{
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};
use nalgebra::{DMatrix, DVector};
use std::sync::Arc;

/// Number of integral states: ξ_x, ξ_y, ξ_z.
const N_INTEGRALS: usize = 3;

/// Model Predictive Controller with integral augmentation.
///
/// Solves a condensed finite-horizon QP at every control step by Cholesky
/// decomposition (projected gradient descent fallback).  The prediction and
/// control horizons are both `horizon` steps.
///
/// # Cost function (per horizon)
///
/// ```text
/// J = Σ_{k=1}^{N} (xk − xref)ᵀ Q (xk − xref)
///   + Σ_{k=0}^{N-1} δukᵀ R δuk
///   + ξᵀ Qi ξ                          (integral penalty)
/// ```
///
/// where `δu = u − u_eq`, `Q = diag(q_weights)`, `R = diag(r_weights)`,
/// and `ξ` is the accumulated position-error integral.
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
    /// Integral cost weights — 3 elements [ξ_x, ξ_y, ξ_z].
    /// Higher values → faster steady-state correction but risk overshoot.
    pub qi_weights: [f64; N_INTEGRALS],
    /// Anti-windup clamp for each integral [m·s].
    pub xi_limits: [f64; N_INTEGRALS],
    /// Accumulated position-error integrals [ξ_x, ξ_y, ξ_z].
    xi: [f64; N_INTEGRALS],
    /// Per-actuator bounds: `u_limits[i] = (lo, hi)`.
    pub u_limits: Vec<(f64, f64)>,
    /// Maximum projected-gradient iterations per solve.
    pub max_iter: usize,
    /// Warm-start: previous optimal control sequence (length N·m), if any.
    prev_u: Option<DVector<f64>>,
    /// Predicted z-positions over the horizon after the last solve.
    /// planned_z[0] = current z, planned_z[k] = predicted z after k MPC steps.
    planned_z: Vec<f64>,
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
        qi_weights: [f64; N_INTEGRALS],
        xi_limits: [f64; N_INTEGRALS],
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
            qi_weights,
            xi_limits,
            xi: [0.0; N_INTEGRALS],
            u_limits,
            max_iter: 150,
            prev_u: None,
            planned_z: Vec::new(),
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
            15.0, 15.0, 50.0, // x y z
            2.0, 2.0, 8.0, // vx vy vz
            4.0, 4.0, 4.0, // ωx ωy ωz
            15.0, 15.0, 15.0, 15.0, // qi qj qk qw
        ];
        let r_weights = vec![0.01; 4];
        let qi_weights = [1.0, 1.0, 3.0]; // integral: ξ_x ξ_y ξ_z
        let xi_limits = [3.0, 3.0, 3.0]; // anti-windup [m·s]
        let u_limits = vec![(0.0, hover_w * 2.0); 4];
        Self::new(model, horizon, dt, q_weights, r_weights, qi_weights, xi_limits, u_limits)
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

    /// Solve the bound-constrained QP `min δUᵀ H δU + fᵀ δU  s.t. lo ≤ δU ≤ hi`.
    ///
    /// Strategy:
    /// 1. Solve the unconstrained system `H δU = −f/2` via Cholesky decomposition.
    ///    This gives the exact optimum instantly, with no step-size or
    ///    convergence issues regardless of the condition number of H.
    /// 2. Clamp each element of the solution to `[lo, hi]`.
    ///
    /// Pure gradient descent needs O(κ) iterations to converge (κ is the
    /// condition number of H).  With the quadrotor’s attitude–position coupling
    /// (A[vz,qw] ≈ 2g) the condition number exceeds 10³, so 150 gradient steps
    /// converge to < 2 % of the optimum — which for a 5 m altitude step
    /// means essentially zero δu and no climb.
    ///
    /// Fallback to projected gradient descent if Cholesky fails (ill-conditioned
    /// or indefinite H, which should not occur for a well-posed quadrotor QP).
    fn solve_bounded(
        h: &DMatrix<f64>,
        f: &DVector<f64>,
        u_lo: &DVector<f64>,
        u_hi: &DVector<f64>,
        max_iter: usize,
    ) -> DVector<f64> {
        // Analytical solve: H·δU = −f/2
        let rhs = -f * 0.5;
        let du_opt = match h.clone().cholesky() {
            Some(chol) => {
                let mut sol = chol.solve(&rhs);
                for i in 0..sol.len() {
                    sol[i] = sol[i].clamp(u_lo[i], u_hi[i]);
                }
                sol
            }
            None => {
                // Fallback: projected gradient descent (slow but always safe).
                let alpha = 1.0 / (2.0 * h.norm() + 1e-8);
                let mut u = DVector::zeros(f.len());
                for _ in 0..max_iter {
                    let grad = 2.0 * (h * &u) + f;
                    u -= alpha * &grad;
                    for i in 0..u.len() {
                        u[i] = u[i].clamp(u_lo[i], u_hi[i]);
                    }
                }
                u
            }
        };
        du_opt
    }
}

impl Controller for MpcController {
    fn update(
        &mut self,
        state: &DroneState,
        target: &FlightTarget,
        _dt: TimeStep,
    ) -> KnownActuatorInput {
        let sim_dt = _dt.seconds();
        let m = self.u_limits.len();
        let n_steps = self.horizon;

        // 0. Accumulate position-error integrals (offset-free MPC).
        //
        // Only integrate axes that are active in the target; absent axes
        // stay frozen so the integral doesn't wind up on uncontrolled axes.
        if let Some(x_ref) = target.x {
            self.xi[0] += (x_ref - state.position.x) * sim_dt;
        }
        if let Some(y_ref) = target.y {
            self.xi[1] += (y_ref - state.position.y) * sim_dt;
        }
        if let Some(z_ref) = target.z {
            self.xi[2] += (z_ref - state.position.z) * sim_dt;
        }
        // Anti-windup clamp.
        for i in 0..N_INTEGRALS {
            self.xi[i] = self.xi[i].clamp(-self.xi_limits[i], self.xi_limits[i]);
        }

        // 1. Linearise and discretise.
        //
        // Use `self.dt` (the MPC's prediction step), NOT the simulation step.
        // Linearise with actuator_state = None: when actuator_state is Some,
        // QuadrotorAero uses the lagged actuator speeds instead of the commanded
        // input, making B = 0 and causing the QP to always return δU = 0.
        let trim_input = self.model.equilibrium_input();
        let state_for_lin = drone_model::state::DroneState {
            actuator_state: None,
            position: state.position,
            velocity: state.velocity,
            orientation: state.orientation,
            angular_velocity: state.angular_velocity,
        };
        let lm = linearize(self.model.as_ref(), &state_for_lin, &trim_input);
        let (ad, bd) = discretize_implicit_euler(&lm.a, &lm.b, self.dt);

        // 2. Condensed Γ matrix (Φ not needed in deviation-coordinate formulation).
        let (_phi, gamma) = Self::build_phi_gamma(&ad, &bd, n_steps);

        // 3. Block-diagonal Q̄ and R̄.
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

        // 4. QP matrices H and f in δU = U − U_eq coordinates.
        //
        // Deviation-coordinate e0: e0[k,i] = x0[i] - x_ref[i] for every
        // horizon step k.  Since we linearise at the current state,
        // δx0 = 0 and the prediction error at δU = 0 is -(x_ref - x0).
        let x0 = state_to_vec(state);
        let x_ref = Self::build_ref(target, state);
        let u0_vec = input_to_vec(&trim_input);

        let n_state = x0.len();
        let e0 = DVector::from_fn(n_steps * n_state, |i, _| x0[i % n_state] - x_ref[i % n_state]);

        let gt_qbar = gamma.transpose() * &q_bar;
        let h = &gt_qbar * &gamma + &r_bar;
        let mut f = 2.0 * &gt_qbar * &e0;

        // 4b. Integral augmentation: add bias to f from accumulated ξ.
        //
        // The integral penalty J_i = qi * ξ_i² doesn't change H (no new
        // decision variables), but its cross-term with the predicted state
        // adds a constant bias to the gradient f.
        //
        // For each integral ξ_i with position state index s_i, the gradient
        // contribution is:  f += 2 * qi_weight[i] * ξ_i * (∂ predicted_pos_s_i / ∂ δU)
        //                     = 2 * qi_weight[i] * ξ_i * Γᵀ_row(s_i) summed over horizon steps.
        //
        // This biases the QP toward commands that reduce the integrated error.
        {
            // Position state indices: x=0, y=1, z=2
            let pos_indices = [0usize, 1, 2];
            for (int_idx, &state_idx) in pos_indices.iter().enumerate() {
                let qi = self.qi_weights[int_idx];
                let xi_val = self.xi[int_idx];
                if qi.abs() < 1e-12 || xi_val.abs() < 1e-12 {
                    continue;
                }
                // Sum Γᵀ columns corresponding to this position state across
                // all horizon steps to get the total sensitivity of the
                // predicted position to each control input.
                let mut grad_col = DVector::zeros(nm);
                for k in 0..n_steps {
                    let row_idx = k * n_state + state_idx;
                    for j in 0..nm {
                        grad_col[j] += gamma[(row_idx, j)];
                    }
                }
                // When ξ > 0 (below target for z), we want more thrust (δU > 0).
                // To push δU positive, f must be negative.  Since grad_col is
                // positive (more motor speed → higher z), we negate the bias.
                // f -= 2 * qi * ξ * grad_col
                f -= (2.0 * qi * xi_val) * &grad_col;
            }
        }

        // 5. Element-wise bounds for δU = U − U_eq.
        let mut du_lo = DVector::zeros(nm);
        let mut du_hi = DVector::zeros(nm);
        for k in 0..n_steps {
            for i in 0..m {
                du_lo[k * m + i] = self.u_limits[i].0 - u0_vec[i];
                du_hi[k * m + i] = self.u_limits[i].1 - u0_vec[i];
            }
        }

        // 6. Solve the bound-constrained QP analytically (Cholesky).
        let du_opt = Self::solve_bounded(&h, &f, &du_lo, &du_hi, self.max_iter);
        self.prev_u = Some(du_opt.clone());

        // 6b. Record the predicted z-trajectory for external inspection.
        self.planned_z.clear();
        self.planned_z.push(x0[2]); // current z (k=0)
        for k in 0..n_steps {
            let row_idx = k * n_state + 2; // z is state index 2
            let delta_z: f64 = gamma
                .row(row_idx)
                .iter()
                .zip(du_opt.iter())
                .map(|(g, d)| g * d)
                .sum();
            self.planned_z.push(x0[2] + delta_z);
        }

        // 7. Apply first control step: u = u_eq + δu.
        let u_first = du_opt.rows(0, m) + u0_vec;
        vec_to_input(&u_first, &trim_input)
    }

    fn reset(&mut self) {
        self.prev_u = None;
        self.planned_z.clear();
        self.xi = [0.0; N_INTEGRALS];
    }

    fn name(&self) -> &str {
        "MPC"
    }

    fn planned_z_horizon(&self) -> Option<&[f64]> {
        if self.planned_z.is_empty() {
            None
        } else {
            Some(&self.planned_z)
        }
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
    fn mpc_below_target_increases_thrust() {
        // Drone at z=0, target z=5 — MPC should increase thrust above hover.
        // This tests the full default config (dt=0.5, N=10) used in simulation.
        let model = Arc::new(QuadrotorModel::mini3_simple());
        let hover_w = match model.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };
        let q = vec![10.0, 10.0, 50.0, 1.0, 1.0, 5.0, 2.0, 2.0, 2.0, 5.0, 5.0, 5.0, 5.0];
        let r = vec![0.1_f64; 4];
        let qi = [0.0, 0.0, 0.0]; // no integral in this test
        let xi_lim = [5.0; 3];
        let u_limits = vec![(0.0, hover_w * 2.0); 4];
        let mut mpc = MpcController::new(model.clone(), 10, 0.5, q, r, qi, xi_lim, u_limits);

        let state = hover_state(0.0);
        let target = FlightTarget::altitude(5.0);
        let dt = TimeStep::constant(0.005);
        let input = mpc.update(&state, &target, dt);

        let u = input_to_vec(&input);
        let u0 = input_to_vec(&model.equilibrium_input());
        let avg = u.mean();
        let avg0 = u0.mean();
        eprintln!("avg={avg:.2}, hover={avg0:.2}, delta={:.2}", avg - avg0);
        assert!(
            avg > avg0,
            "Below target z=0->5: avg speed {avg:.2} should be > hover {avg0:.2}"
        );
    }

    #[test]
    fn solve_bounded_unconstrained_min() {
        // Minimise J = u² − 2u  ⇒  H = [[1]], f = [[−2]], optimal u = 1
        let h = DMatrix::from_element(1, 1, 1.0);
        let f = DVector::from_element(1, -2.0);
        let lo = DVector::from_element(1, -100.0);
        let hi = DVector::from_element(1, 100.0);
        let u = MpcController::solve_bounded(&h, &f, &lo, &hi, 500);
        assert!(
            (u[0] - 1.0).abs() < 1e-10,
            "optimal u should = 1, got {:.6}",
            u[0]
        );
    }
}
