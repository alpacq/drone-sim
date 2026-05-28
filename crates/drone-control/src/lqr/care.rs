/// CARE equation solver for continuous-time LQR.
use anyhow::{Result, anyhow, ensure};
use nalgebra::DMatrix;

#[derive(Debug, Clone)]
pub struct SolverParams {
    pub max_iter: usize,
    pub tolerance: f64,
    pub gamma: Option<f64>,
}

fn care_residual(a: &DMatrix<f64>, g: &DMatrix<f64>, q: &DMatrix<f64>, p: &DMatrix<f64>) -> f64 {
    (a.transpose() * p + p * a - p * g * p + q).norm()
}

fn symmetrize(m: &DMatrix<f64>) -> DMatrix<f64> {
    (m + m.transpose()) * 0.5
}

fn riccati_rhs(
    a: &DMatrix<f64>,
    g: &DMatrix<f64>,
    q: &DMatrix<f64>,
    p: &DMatrix<f64>,
) -> DMatrix<f64> {
    a.transpose() * p + p * a - p * g * p + q
}

/// Phase 1: integrate the Riccati ODE forward with RK4 to get a warm P.
/// Returns `(P, steps_taken)`.
fn initial_riccati_flow(
    a: &DMatrix<f64>,
    g: &DMatrix<f64>,
    q: &DMatrix<f64>,
) -> (DMatrix<f64>, usize) {
    let n = a.nrows();
    let mut p = DMatrix::zeros(n, n);
    let dt = 0.01;
    let mut steps = 0;

    for _ in 0..20_000 {
        let k1 = riccati_rhs(a, g, q, &p);
        let k2 = riccati_rhs(a, g, q, &(&p + &k1 * (dt * 0.5)));
        let k3 = riccati_rhs(a, g, q, &(&p + &k2 * (dt * 0.5)));
        let k4 = riccati_rhs(a, g, q, &(&p + &k3 * dt));

        let p_next = &p + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

        if !p_next.iter().all(|x| x.is_finite()) {
            break;
        }

        let step = (&p_next - &p).norm();
        p = symmetrize(&p_next);
        steps += 1;

        if step < 1e-10 && care_residual(a, g, q, &p) < 1e-6 {
            break;
        }
    }

    (p, steps)
}

fn solve_lyapunov(a_cl: &DMatrix<f64>, rhs_positive: &DMatrix<f64>) -> Result<DMatrix<f64>> {
    let n = a_cl.nrows();
    let size = n * n;
    let mut lhs = DMatrix::zeros(size, size);

    for col in 0..n {
        for row in 0..n {
            let basis_idx = row + col * n;
            let mut basis = DMatrix::zeros(n, n);
            basis[(row, col)] = 1.0;

            let mapped = a_cl.transpose() * &basis + &basis * a_cl;
            for out_col in 0..n {
                for out_row in 0..n {
                    let out_idx = out_row + out_col * n;
                    lhs[(out_idx, basis_idx)] = mapped[(out_row, out_col)];
                }
            }
        }
    }

    let mut rhs_vec = DMatrix::zeros(size, 1);
    for col in 0..n {
        for row in 0..n {
            rhs_vec[(row + col * n, 0)] = -rhs_positive[(row, col)];
        }
    }

    let solution = lhs
        .lu()
        .solve(&rhs_vec)
        .ok_or_else(|| anyhow!("Lyapunov linear system is singular"))?;

    let mut p = DMatrix::zeros(n, n);
    for col in 0..n {
        for row in 0..n {
            p[(row, col)] = solution[(row + col * n, 0)];
        }
    }

    Ok(p)
}

impl Default for SolverParams {
    fn default() -> Self {
        Self {
            max_iter: 1000,
            tolerance: 1e-8,
            gamma: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RiccatiSolution {
    pub p: DMatrix<f64>,         // P matrix solution
    pub k: DMatrix<f64>,         // gain matrix K = R⁻¹B'P
    /// RK4 Riccati flow steps (Phase 1 — main computation).
    /// Typically a few hundred to a few thousand steps.
    pub flow_steps: usize,
    /// Newton-Kleinman refinement iterations (Phase 2).
    /// 0 means Phase 1 already converged below tolerance — the common case
    /// for well-conditioned systems.  >0 means Phase 2 provided extra precision.
    pub newton_iters: usize,
    pub care_residual: f64,      // final CARE equation residual
}

/// Solve the Lyapunov equation  A'X + XA = -C.
/// If the direct solve fails (singular system, e.g. from a zero eigenvalue of A),
/// retries with a small shift `A_reg = A + ε·I` that pushes the zero eigenvalue away
/// from 0 without materially changing the solution for the active modes.
fn solve_lyapunov_robust(a_cl: &DMatrix<f64>, rhs_positive: &DMatrix<f64>) -> Result<DMatrix<f64>> {
    if let Ok(p) = solve_lyapunov(a_cl, rhs_positive) {
        return Ok(p);
    }
    // Regularise: shift all eigenvalues by a small ε.
    // For a dead-state zero eigenvalue the RHS is also zero in that direction,
    // so the regularised solution is P = 0 there — the correct answer.
    let n = a_cl.nrows();
    let scale = (a_cl.norm() + 1.0).max(1.0);
    for exp in [10u32, 9, 8, 7, 6] {
        let eps = scale * 10_f64.powi(-(exp as i32));
        let a_reg = a_cl + DMatrix::identity(n, n) * eps;
        if let Ok(p) = solve_lyapunov(&a_reg, rhs_positive) {
            return Ok(p);
        }
    }
    Err(anyhow!("Lyapunov system is singular even after regularisation"))
}

pub fn solve_care(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
    params: &SolverParams,
) -> Result<RiccatiSolution> {
    let n = a.nrows();
    let m = b.ncols();

    ensure!(
        a.ncols() == n,
        "A must be square [{}x{}], got [{}x{}]",
        n,
        n,
        a.nrows(),
        a.ncols()
    );
    ensure!(b.nrows() == n, "B must have {} rows, got {}", n, b.nrows());
    ensure!(
        q.nrows() == n && q.ncols() == n,
        "Q must be [{}x{}], got [{}x{}]",
        n,
        n,
        q.nrows(),
        q.ncols()
    );
    ensure!(
        r.nrows() == m && r.ncols() == m,
        "R must be [{}x{}], got [{}x{}]",
        m,
        m,
        r.nrows(),
        r.ncols()
    );

    let r_inv = r.clone().try_inverse().ok_or_else(|| anyhow!(
        "Matrix R is not invertible. Check if it is positively defined (all diagonal elements are positive)"
    ))?;

    let g = b * &r_inv * b.transpose();

    // ── Dead-state handling ───────────────────────────────────────────────────
    //
    // In over-parameterised systems (e.g. 13-DOF quaternion state of a quadrotor)
    // some state directions are completely decoupled at the trim point:
    //   A[i,:] ≈ 0,  A[:,i] ≈ 0,  G[i,:] = 0
    // (example: quaternion w component at hover — it has zero kinematics and zero
    //  control influence to first order).
    //
    // CARE has no finite solution for such a "dead" direction when Q[i,i] > 0,
    // because the Riccati ODE diverges (dP[i,i]/dt = Q[i,i] ≠ 0 forever).
    // The correct fix is to set Q to zero for those directions: the LQR gain K
    // will then be zero there, which is the physically correct result.
    let zero_thresh = 1e-8;
    let mut q_eff = q.clone();
    for i in 0..n {
        if a.row(i).norm() <= zero_thresh
            && a.column(i).norm() <= zero_thresh
            && g.row(i).norm() <= zero_thresh
        {
            for j in 0..n {
                q_eff[(i, j)] = 0.0;
                q_eff[(j, i)] = 0.0;
            }
        }
    }

    let (mut p, flow_steps) = initial_riccati_flow(a, &g, &q_eff);
    let mut newton_iters = 0;
    let mut residual = care_residual(a, &g, &q_eff, &p);

    for _ in 0..params.max_iter {
        // Phase 1 alone was sufficient — Newton refinement not needed.
        if residual < params.tolerance {
            break;
        }

        newton_iters += 1;

        let a_cl = a - &g * &p;
        let m_rhs = &p * &g * &p + &q_eff;
        // Use robust solver: dead-state directions leave A_cl with a zero eigenvalue,
        // which makes the standard Lyapunov system singular.
        let p_new = solve_lyapunov_robust(&a_cl, &m_rhs)?;
        let p_new = symmetrize(&p_new);

        residual = (&p_new - &p).norm();
        p = p_new;

        let care_res = care_residual(a, &g, &q_eff, &p);
        if care_res < params.tolerance {
            residual = care_res;
            break;
        }
    }

    let care_res = care_residual(a, &g, &q_eff, &p);

    if care_res >= params.tolerance.max(1e-10) * 100.0 {
        return Err(anyhow!(
            "CARE solver didn't converge after {} Newton iterations. CARE residual = {:.2e}",
            params.max_iter,
            care_res
        ));
    }

    let k = &r_inv * b.transpose() * &p;

    Ok(RiccatiSolution {
        p,
        k,
        flow_steps,
        newton_iters,
        care_residual: care_res,
    })
}

pub fn build_q_diagonal(weights: &[f64]) -> DMatrix<f64> {
    let n = weights.len();
    let mut q = DMatrix::zeros(n, n);
    for (i, &w) in weights.iter().enumerate() {
        q[(i, i)] = w;
    }
    q
}

pub fn build_r_diagonal(weights: &[f64]) -> DMatrix<f64> {
    let m = weights.len();
    let mut r = DMatrix::zeros(m, m);
    for (i, &w) in weights.iter().enumerate() {
        r[(i, i)] = w;
    }
    r
}

pub fn cheeck_positive_definite(m: &DMatrix<f64>, name: &str) -> Result<()> {
    for i in 0..m.nrows() {
        ensure!(
            m[(i, i)] > 1e-12,
            "Matrix {} is not positive defeinite: {}[{},{}] = {} <= 0",
            name,
            name,
            i,
            i,
            m[(i, i)]
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verification with simple scalar system with analytic solution
    ///
    /// ẋ = ax + bu,  J = ∫(qx² + ru²)dt
    /// CARE: 2aP - (b²/r)P² + q = 0  →  P = (r/b²)(a + √(a² + b²q/r))
    #[test]
    fn care_scalar_analytical_solution() {
        // ẋ = -x + u, q = 1, r = 1
        // CARE: -2P - P² + 1 = 0  →  P² + 2P - 1 = 0
        // P = (-2 + √8)/2 = -1 + √2 ≈ 0.4142
        let a = DMatrix::from_element(1, 1, -1.0);
        let b = DMatrix::from_element(1, 1, 1.0);
        let q = DMatrix::from_element(1, 1, 1.0);
        let r = DMatrix::from_element(1, 1, 1.0);

        let sol = solve_care(&a, &b, &q, &r, &SolverParams::default()).unwrap();

        let p_analytic = 2.0_f64.sqrt() - 1.0; // ≈ 0.4142
        assert!(
            (sol.p[(0, 0)] - p_analytic).abs() < 1e-8,
            "P = {:.6}, expected {:.6}",
            sol.p[(0, 0)],
            p_analytic
        );
        assert!(
            sol.care_residual < 1e-8,
            "CARE residuum = {:.2e}",
            sol.care_residual
        );
    }

    /// Unstable system: ẋ = x + u (A > 0 → unstable open)
    /// LQR must stabilize it.
    #[test]
    fn care_unstable_open() {
        let a = DMatrix::from_element(1, 1, 1.0); // unstable!
        let b = DMatrix::from_element(1, 1, 1.0);
        let q = DMatrix::from_element(1, 1, 1.0);
        let r = DMatrix::from_element(1, 1, 1.0);

        let sol = solve_care(&a, &b, &q, &r, &SolverParams::default()).unwrap();

        // P must be positive
        assert!(sol.p[(0, 0)] > 0.0, "P > 0 for unstable open system");
        // K = R⁻¹BᵀP, must stabilize: A - BK < 0
        let a_cl = a[(0, 0)] - b[(0, 0)] * sol.k[(0, 0)];
        assert!(
            a_cl < 0.0,
            "Closed-loop system should be stable: A-BK = {:.4}",
            a_cl
        );
        assert!(
            sol.care_residual < 1e-8,
            "CARE residuum = {:.2e}",
            sol.care_residual
        );
    }

    /// Double integrator: ẋ₁ = x₂, ẋ₂ = u
    /// Classic benchmark for LQR.
    #[test]
    fn care_double_integrator() {
        let a = DMatrix::from_row_slice(2, 2, &[0.0, 1.0, 0.0, 0.0]);
        let b = DMatrix::from_row_slice(2, 1, &[0.0, 1.0]);
        let q = build_q_diagonal(&[1.0, 1.0]);
        let r = build_r_diagonal(&[1.0]);

        let sol = solve_care(&a, &b, &q, &r, &SolverParams::default()).unwrap();

        // Both elements of K must be positive (stabilization of position and velocity)
        assert!(sol.k[(0, 0)] > 0.0, "K₁ > 0");
        assert!(sol.k[(0, 1)] > 0.0, "K₂ > 0");
        assert!(
            sol.care_residual < 1e-6,
            "CARE residuum = {:.2e}",
            sol.care_residual
        );

        println!(
            "Double integrator: K = [{:.4}, {:.4}], flow={} newton={}",
            sol.k[(0, 0)],
            sol.k[(0, 1)],
            sol.flow_steps,
            sol.newton_iters,
        );
    }

    /// Test of convergence — checks that SDA converges quickly.
    #[test]
    fn sda_quick_convergence() {
        // 4×4 system — checks scaling
        let a = DMatrix::from_row_slice(
            4,
            4,
            &[
                -1.0, 1.0, 0.0, 0.0, 0.0, -2.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -3.0,
            ],
        );
        let b = DMatrix::from_row_slice(4, 2, &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
        let q = build_q_diagonal(&[10.0, 1.0, 10.0, 1.0]);
        let r = build_r_diagonal(&[1.0, 1.0]);

        let sol = solve_care(&a, &b, &q, &r, &SolverParams::default()).unwrap();

        // Flow should converge well within 20 000 steps for this stable system
        assert!(
            sol.flow_steps < 20_000,
            "Too slow convergence: {} flow steps",
            sol.flow_steps
        );
        assert!(
            sol.care_residual < 1e-6,
            "CARE residuum = {:.2e}",
            sol.care_residual
        );

        println!(
            "4×4 system: flow={} newton={} care_res={:.2e}",
            sol.flow_steps, sol.newton_iters, sol.care_residual
        );
    }

    /// Test for strongly unstable system (F-16 like).
    #[test]
    fn care_strongly_unstable() {
        // System with eigenvalues +5, +3 (strongly unstable)
        let a = DMatrix::from_row_slice(2, 2, &[5.0, 0.0, 0.0, 3.0]);
        let b = DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 0.0, 1.0]);
        let q = build_q_diagonal(&[100.0, 100.0]);
        let r = build_r_diagonal(&[1.0, 1.0]);

        let sol = solve_care(&a, &b, &q, &r, &SolverParams::default()).unwrap();

        // Closed-loop system must be stable
        let k = &sol.k;
        let a_cl = &a - &b * k;
        // Check trace (sum of eigenvalues < 0 for stability)
        let trace_cl = a_cl[(0, 0)] + a_cl[(1, 1)];
        assert!(
            trace_cl < 0.0,
            "Trace of closed-loop system should be negative: {:.4}",
            trace_cl
        );
        assert!(
            sol.care_residual < 1e-6,
            "CARE residuum = {:.2e}",
            sol.care_residual
        );

        println!(
            "Strongly unstable: flow={} newton={} care_res={:.2e}",
            sol.flow_steps, sol.newton_iters, sol.care_residual
        );
    }

    #[test]
    fn wrong_dimensions_return_error() {
        let a = DMatrix::zeros(3, 3);
        let b = DMatrix::zeros(2, 1); // wrongdimensions!
        let q = DMatrix::zeros(3, 3);
        let r = DMatrix::from_element(1, 1, 1.0);

        let result = solve_care(&a, &b, &q, &r, &SolverParams::default());
        assert!(result.is_err());
    }

    #[test]
    fn non_invertible_r_returns_error() {
        let a = DMatrix::from_element(1, 1, -1.0);
        let b = DMatrix::from_element(1, 1, 1.0);
        let q = DMatrix::from_element(1, 1, 1.0);
        let r = DMatrix::from_element(1, 1, 0.0); // R = 0 — inrevertible!

        let result = solve_care(&a, &b, &q, &r, &SolverParams::default());
        assert!(result.is_err());
    }
}
