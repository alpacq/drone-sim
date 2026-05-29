use drone_model::{
    math::euler::quat_to_euler,
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};
/// Linear-Quadratic Integral (LQI) controller.
///
/// Extends LQR with 4 integral states [ξ_x, ξ_y, ξ_z, ξ_ψ] to eliminate
/// steady-state tracking error caused by model mismatch and persistent
/// disturbances (e.g. drag, battery sag, constant wind).
///
/// # Augmented state
///
/// ```text
/// z = [δx (13D plant deviation from trim); ξ (4D integrals)]  →  17D total
/// ```
///
/// The gain matrix K ∈ ℝ^(m×17) is computed once by solving the CARE on the
/// augmented system and never changes at runtime.
///
/// # Universal operation
///
/// At runtime only the integrals for *active* FlightTarget axes are updated;
/// the rest are frozen (ξ̇ = 0).  Because K always has the same shape, one
/// design works for every FlightTarget configuration without re-solving CARE.
///
/// | FlightTarget active    | ξ̇ updated | ξ̇ frozen  |
/// |------------------------|-----------|-----------|
/// | `position = Some(p)`  | ξ_x ξ_y ξ_z | —       |
/// | `yaw = Some(ψ)`       | ξ_ψ       | —         |
/// | `position = None`     | —         | ξ_x ξ_y ξ_z |
/// | `yaw = None`          | —         | ξ_ψ       |
use nalgebra::{DMatrix, DVector};
use thiserror::Error;

use crate::{
    controller::Controller,
    lqr::{
        care::{CareError, SolverParams, build_r_diagonal, solve_care},
        linearize::{linearize, state_to_vec, vec_to_input},
    },
    target::FlightTarget,
};

/// Errors returned by [`LqiController::design`].
#[derive(Debug, Error)]
pub enum LqiError {
    #[error("c_int must be {n_integrals}×{n_plant} (N_INTEGRALS × n_plant); got {rows}×{cols}")]
    WrongCIntShape {
        n_integrals: usize,
        n_plant: usize,
        rows: usize,
        cols: usize,
    },

    #[error(
        "q_weights must have {expected} elements ({n_plant} plant + {n_integrals} integrals); got {actual}"
    )]
    WrongQWeightsLen {
        expected: usize,
        n_plant: usize,
        n_integrals: usize,
        actual: usize,
    },

    /// The underlying CARE solver failed.
    #[error("CARE solver: {0}")]
    Care(#[from] CareError),
}

/// Number of integral states — always 4, fixed at compile time.
/// Indices: 0 = ξ_x, 1 = ξ_y, 2 = ξ_z, 3 = ξ_ψ
const N_INTEGRALS: usize = 4;

pub struct LqiController {
    /// Full gain matrix  K ∈ ℝ^(m × (n_plant + 4))
    k: DMatrix<f64>,
    /// Trim state vector (plant, 13D for quadrotor)
    x0: DVector<f64>,
    /// Trim control vector (equilibrium input)
    u0: DVector<f64>,
    /// Integral states [ξ_x, ξ_y, ξ_z, ξ_ψ]
    xi: [f64; N_INTEGRALS],
    /// Anti-windup clamp for each integral [m·s, m·s, m·s, rad·s]
    pub xi_limits: [f64; N_INTEGRALS],
    input_template: KnownActuatorInput,
    u_limits: Vec<(f64, f64)>,
}

impl LqiController {
    /// Design an LQI controller linearised around `trim_state`.
    ///
    /// `c_int` — output selection matrix `(N_INTEGRALS × n_plant)` that maps
    /// plant states to the integrated outputs.  Use `quadrotor_c_integral(n)`
    /// for the standard quadrotor configuration (x, y, z, yaw).
    ///
    /// `q_weights` must have length `n_plant + N_INTEGRALS` (17 for quadrotor):
    ///   - indices 0..n_plant : weights on plant state deviations
    ///   - indices n_plant..  : weights on [ξ_x, ξ_y, ξ_z, ξ_ψ]
    ///
    /// Typical integral weights are in the range 5–50; higher values give
    /// faster steady-state correction but risk integrator-induced oscillation.
    pub fn design(
        model: &dyn VehicleModel,
        trim_state: &DroneState,
        c_int: DMatrix<f64>,
        q_weights: &[f64],
        r_weights: &[f64],
        u_limits: Vec<(f64, f64)>,
    ) -> Result<Self, LqiError> {
        let trim_input = model.equilibrium_input();
        let linearized = linearize(model, trim_state, &trim_input);

        let n = linearized.a.nrows(); // plant state dim  (13 for quadrotor)
        let m = linearized.b.ncols(); // control input dim (4 for quadrotor)
        let n_aug = n + N_INTEGRALS; // augmented state dim (17)

        if c_int.nrows() != N_INTEGRALS || c_int.ncols() != n {
            return Err(LqiError::WrongCIntShape {
                n_integrals: N_INTEGRALS,
                n_plant: n,
                rows: c_int.nrows(),
                cols: c_int.ncols(),
            });
        }
        if q_weights.len() != n_aug {
            return Err(LqiError::WrongQWeightsLen {
                expected: n_aug,
                n_plant: n,
                n_integrals: N_INTEGRALS,
                actual: q_weights.len(),
            });
        }

        // ── Augmented A matrix (n_aug × n_aug) ──────────────────────────────
        //
        //   ┌           ┐
        //   │ A   │  0  │   ← plant: δẋ = A·δx + B·u
        //   ├─────┼─────┤
        //   │ -C  │  0  │   ← integrals: ξ̇ = –C·δx  (reference is added
        //   └           ┘                              at runtime, not here)

        let mut a_aug = DMatrix::zeros(n_aug, n_aug);
        a_aug.view_mut((0, 0), (n, n)).copy_from(&linearized.a);
        a_aug
            .view_mut((n, 0), (N_INTEGRALS, n))
            .copy_from(&(-&c_int));

        // ── Augmented B matrix (n_aug × m) ──────────────────────────────────
        //
        //   ┌   ┐
        //   │ B │   ← plant input
        //   ├───┤
        //   │ 0 │   ← integrals are not directly controlled
        //   └   ┘
        let mut b_aug = DMatrix::zeros(n_aug, m);
        b_aug.view_mut((0, 0), (n, m)).copy_from(&linearized.b);

        // ── Q and R ─────────────────────────────────────────────────────────
        let mut q_aug = DMatrix::zeros(n_aug, n_aug);
        for (i, &w) in q_weights.iter().enumerate() {
            q_aug[(i, i)] = w;
        }
        let r = build_r_diagonal(r_weights);

        // ── Solve CARE on augmented system ──────────────────────────────────
        let solution = solve_care(&a_aug, &b_aug, &q_aug, &r, &SolverParams::default())?;

        // Diagnostics available via solution.flow_steps / solution.newton_iters
        // if the caller wants them.

        Ok(Self {
            k: solution.k,
            x0: linearized.x0,
            u0: linearized.u0,
            xi: [0.0; N_INTEGRALS],
            // Default anti-windup limits.  Public so callers can adjust.
            // Anti-windup limits: reduced from 30 m·s to 10 m·s for xyz so
            // the integrator cannot wind up as much during a large initial step
            // (e.g. 0 → 5 m), which would otherwise cause significant overshoot
            // when the drone crosses the setpoint while ξ is still large.
            // 10 m·s allows the integrator to accumulate ~10 s of 1 m error,
            // which is still more than enough to reject constant disturbances.
            xi_limits: [10.0, 10.0, 10.0, std::f64::consts::PI * 2.0],
            input_template: trim_input,
            u_limits,
        })
    }

    /// Integrate tracking errors for axes that are active in `target`.
    /// Axes absent from `target` have their integrators frozen (ξ̇ = 0).
    fn update_integrals(&mut self, state: &DroneState, target: &FlightTarget, dt: f64) {
        // Integrate only the axes present in the target; absent axes stay frozen.
        if let Some(x) = target.x { self.xi[0] += (x - state.position.x) * dt; }
        if let Some(y) = target.y { self.xi[1] += (y - state.position.y) * dt; }
        if let Some(z) = target.z { self.xi[2] += (z - state.position.z) * dt; }
        if let Some(yaw_ref) = target.yaw {
            let euler = quat_to_euler(&state.orientation);
            let err = normalize_angle(yaw_ref - euler.yaw);
            self.xi[3] += err * dt;
        }

        // Anti-windup: clamp each integral to its limit.
        for (xi, &lim) in self.xi.iter_mut().zip(self.xi_limits.iter()) {
            *xi = xi.clamp(-lim, lim);
        }
    }

    /// Compute control output from current state and accumulated integrals.
    fn compute_control(&self, state: &DroneState) -> DVector<f64> {
        let x = state_to_vec(state);
        let n = x.len();

        // Augmented state deviation: z = [δx; ξ]
        let mut z = DVector::zeros(n + N_INTEGRALS);
        for i in 0..n {
            z[i] = x[i] - self.x0[i]; // δx
        }
        for i in 0..N_INTEGRALS {
            z[n + i] = self.xi[i]; // ξ
        }

        // u = u0 - K·z   (K is positive by CARE convention)
        let u_raw = &self.u0 - &self.k * &z;

        let mut u = u_raw;
        for i in 0..u.len().min(self.u_limits.len()) {
            let (lo, hi) = self.u_limits[i];
            u[i] = u[i].clamp(lo, hi);
        }
        u
    }
}

/// Standard output selection matrix for a quadrotor LQI controller.
///
/// Returns C \in R^(4 x n_plant) that integrates:
///   \xi_x <- x  (index 0),  \xi_y <- y  (index 1),  \xi_z <- z  (index 2)
///   \xi_psi <- 2*qz at hover  (yaw linearisation; qz = state index 11)
///
/// For a custom vehicle or non-standard state layout, build your own C matrix
/// and pass it to `LqiController::design` directly.
pub fn quadrotor_c_integral(n_plant: usize) -> DMatrix<f64> {
    let mut c = DMatrix::zeros(N_INTEGRALS, n_plant);
    if n_plant > 2 {
        c[(0, 0)] = 1.0; // x
        c[(1, 1)] = 1.0; // y
        c[(2, 2)] = 1.0; // z
    }
    if n_plant > 11 {
        // Linearised yaw around hover (identity quaternion):
        //   yaw = atan2(2(qw*qz + qx*qy), ...)  ->  d(yaw)/d(qz)|_{q=I} = 2
        // qz = q.k = state index 11 (see linearize::state_to_vec)
        c[(3, 11)] = 2.0;
    }
    c
}

fn normalize_angle(a: f64) -> f64 {
    let mut a = a;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

impl Controller for LqiController {
    fn update(
        &mut self,
        state: &DroneState,
        target: &FlightTarget,
        dt: TimeStep,
    ) -> KnownActuatorInput {
        self.update_integrals(state, target, dt.seconds());
        let u = self.compute_control(state);
        vec_to_input(&u, &self.input_template)
    }

    fn reset(&mut self) {
        self.xi = [0.0; N_INTEGRALS];
    }

    fn name(&self) -> &str {
        "LqiController"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{state::DroneState, vehicle::quadrotor::QuadrotorModel};
    use nalgebra::{UnitQuaternion, Vector3};

    fn hover_state() -> DroneState {
        DroneState {
            position: Vector3::new(0.0, 0.0, 5.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    fn make_lqi(model: &QuadrotorModel) -> LqiController {
        let hover = hover_state();
        let n_plant = 13; // quadrotor
        let c_int = quadrotor_c_integral(n_plant);
        // 13 plant weights + 4 integral weights
        let q_weights: Vec<f64> = [
            // plant: position, velocity, angular velocity, quaternion
            10.0, 10.0, 50.0, // x y z
            1.0, 1.0, 5.0, // vx vy vz
            2.0, 2.0, 2.0, // ωx ωy ωz
            20.0, 20.0, 20.0, 20.0, // qx qy qz qw
            // integrals: ξ_x ξ_y ξ_z ξ_ψ
            5.0, 5.0, 20.0, 2.0,
        ]
        .into();
        let r_weights = vec![0.1; 4];
        let hover_w = match model.equilibrium_input() {
            drone_model::vehicle::KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };
        LqiController::design(
            model,
            &hover,
            c_int,
            &q_weights,
            &r_weights,
            vec![(0.0, hover_w * 2.0); 4],
        )
        .expect("LQI design failed")
    }

    #[test]
    fn lqi_designs_successfully() {
        let model = QuadrotorModel::mini3_simple();
        let ctrl = make_lqi(&model);

        // K must have shape (4 motors) × (13 plant + 4 integrals = 17)
        assert_eq!(ctrl.k.nrows(), 4, "K rows = number of motors");
        assert_eq!(ctrl.k.ncols(), 17, "K cols = plant + integral states");
        assert!(ctrl.k.norm() > 0.01, "K should be non-trivial");
    }

    #[test]
    fn wrong_q_weight_length_returns_error() {
        let model = QuadrotorModel::mini3_simple();
        let hover = hover_state();
        let c_int = quadrotor_c_integral(13);
        // Only 13 weights (missing 4 integral weights)
        let q_weights = vec![1.0; 13];
        let r_weights = vec![0.1; 4];
        let result = LqiController::design(&model, &hover, c_int, &q_weights, &r_weights, vec![]);
        assert!(result.is_err(), "Should fail with wrong q_weights length");
    }

    #[test]
    fn yaw_integral_frozen_when_not_in_target() {
        let model = QuadrotorModel::mini3_simple();
        let mut ctrl = make_lqi(&model);

        // Altitude-only target → yaw NOT active
        let target = FlightTarget::altitude(5.0);
        let state = hover_state();
        ctrl.update_integrals(&state, &target, 1.0);

        assert_eq!(ctrl.xi[3], 0.0, "ξ_ψ must stay 0 when yaw not in target");
    }

    #[test]
    fn position_integrals_frozen_when_not_in_target() {
        let model = QuadrotorModel::mini3_simple();
        let mut ctrl = make_lqi(&model);

        // No position in target — all xyz integrals must stay frozen.
        let target = FlightTarget { x: None, y: None, z: None, yaw: None };
        let state = hover_state();
        ctrl.update_integrals(&state, &target, 1.0);

        assert_eq!(ctrl.xi[0], 0.0, "ξ_x frozen");
        assert_eq!(ctrl.xi[1], 0.0, "ξ_y frozen");
        assert_eq!(ctrl.xi[2], 0.0, "ξ_z frozen");
    }

    #[test]
    fn altitude_integral_accumulates_below_target() {
        let model = QuadrotorModel::mini3_simple();
        let mut ctrl = make_lqi(&model);

        // Drone at z=4, target z=5 → error = +1 m → ξ_z grows positive
        let state = DroneState {
            position: Vector3::new(0.0, 0.0, 4.0),
            ..hover_state()
        };
        let target = FlightTarget::altitude(5.0);
        ctrl.update_integrals(&state, &target, 0.1); // dt = 0.1 s

        assert!(
            ctrl.xi[2] > 0.0,
            "ξ_z should be positive when drone is below target"
        );
        assert!(
            (ctrl.xi[2] - 0.1).abs() < 1e-10,
            "ξ_z = 1 m × 0.1 s = 0.1 m·s, got {}",
            ctrl.xi[2]
        );
    }

    #[test]
    fn anti_windup_clamps_integral() {
        let model = QuadrotorModel::mini3_simple();
        let mut ctrl = make_lqi(&model);

        // Force a very large integral by simulating 1000 s of 1 m error
        let state = DroneState {
            position: Vector3::new(0.0, 0.0, 4.0),
            ..hover_state()
        };
        let target = FlightTarget::altitude(5.0);
        ctrl.update_integrals(&state, &target, 1000.0);

        assert!(
            ctrl.xi[2] <= ctrl.xi_limits[2],
            "ξ_z must be clamped to limit {}, got {}",
            ctrl.xi_limits[2],
            ctrl.xi[2]
        );
    }

    #[test]
    fn reset_clears_all_integrals() {
        let model = QuadrotorModel::mini3_simple();
        let mut ctrl = make_lqi(&model);

        let state = DroneState {
            position: Vector3::new(1.0, 2.0, 3.0),
            ..hover_state()
        };
        let target = FlightTarget::full(0.0, 0.0, 5.0, 0.5);
        ctrl.update_integrals(&state, &target, 1.0);

        ctrl.reset();

        assert_eq!(ctrl.xi, [0.0; 4], "All integrals must be zero after reset");
    }

    #[test]
    fn at_trim_control_close_to_u0() {
        let model = QuadrotorModel::mini3_simple();
        let mut ctrl = make_lqi(&model);

        // At the trim point (z=5, ξ=0): output must be ≈ u0
        let state = hover_state();
        let target = FlightTarget::altitude(5.0);
        let input = ctrl.update(&state, &target, TimeStep::constant(0.005));

        let u_vec = crate::lqr::linearize::input_to_vec(&input);
        let u0_norm = ctrl.u0.norm();
        let u_norm = u_vec.norm();

        assert!(
            (u_norm - u0_norm).abs() / u0_norm < 0.05,
            "At trim: u ≈ u0 (got {:.3}, expected {:.3})",
            u_norm,
            u0_norm
        );
    }
}
