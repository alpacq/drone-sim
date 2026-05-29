//! Longitudinal trim solver for the F-16 model.
//!
//! Finds `(throttle, elevator, alpha)` that produce steady, wings-level,
//! horizontal flight at a given speed and altitude by minimising the squared
//! norm of `(acceleration, angular_acceleration)` using Nelder-Mead simplex
//! optimisation.

use super::F16Model;
use crate::state::DroneState;
use crate::time::TimeStep;
use crate::vehicle::{KnownActuatorInput, VehicleModel};
use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

/// Result of a successful trim search.
#[derive(Debug, Clone)]
pub struct TrimResult {
    /// Trim angle of attack [rad].
    pub alpha_rad: f64,
    /// Trim throttle setting [0, 1].
    pub throttle: f64,
    /// Trim elevator (normalised to [-1, 1], maps to [-25°, +25°]).
    pub elevator: f64,
    /// Residual norm of derivatives at trim state [m/s² equivalent].
    pub residual: f64,
}

/// Errors returned by [`find_trim`].
#[derive(Debug, Error)]
pub enum TrimError {
    /// The optimisation did not reach an acceptable residual.
    #[error("Trim search did not converge: residual {residual:.4} after {iters} iterations")]
    NoConvergence {
        /// Residual norm at the best point found.
        residual: f64,
        /// Number of iterations executed.
        iters: usize,
    },
}

/// Find the longitudinal trim state for horizontal flight at the given speed and altitude.
///
/// Searches for `(throttle, elevator, alpha_deg)` that minimises the squared
/// norm of `(acceleration, angular_acceleration)` at the candidate state using
/// Nelder-Mead simplex optimisation.
///
/// Returns `Err(TrimError::NoConvergence)` if the residual exceeds `tol` after
/// `max_iter` iterations.
///
/// # Example
/// ```no_run
/// use drone_model::vehicle::fixed_wing::f16::{F16Model, trim::find_trim};
/// let model = F16Model::f16a();
/// let trim = find_trim(&model, 200.0, 0.0).unwrap();
/// println!("α = {:.2}°, throttle = {:.3}, elevator = {:.3}",
///          trim.alpha_rad.to_degrees(), trim.throttle, trim.elevator);
/// ```
pub fn find_trim(
    _model: &F16Model,
    speed_ms: f64,
    altitude_m: f64,
) -> Result<TrimResult, TrimError> {
    let max_iter = 500;
    let tol = 50.0; // acceptable squared residual — sum of accel² + ang_accel²

    let bounds = vec![(0.05_f64, 1.0), (-1.0_f64, 1.0), (-5.0_f64, 25.0)];
    let x0 = vec![0.75_f64, 0.0, 1.0];
    // Per-dimension steps: ~20% of each dimension's range
    let steps = vec![0.2, 0.2, 3.0];

    let obj = |x: &[f64]| -> f64 {
        let throttle = x[0].clamp(bounds[0].0, bounds[0].1);
        let elevator = x[1].clamp(bounds[1].0, bounds[1].1);
        let alpha_deg = x[2].clamp(bounds[2].0, bounds[2].1);

        let alpha_rad = alpha_deg.to_radians();
        let state = DroneState {
            position: Vector3::new(0.0, 0.0, altitude_m.max(0.0)),
            velocity: Vector3::new(speed_ms, 0.0, 0.0),
            orientation: UnitQuaternion::from_euler_angles(0.0, -alpha_rad, 0.0),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };
        let input = KnownActuatorInput::FixedWing {
            throttle,
            aileron: 0.0,
            elevator,
            rudder: 0.0,
        };

        // Fresh model + warm engine
        let model = F16Model::f16a();
        let wdt = TimeStep::constant(0.01);
        let mut warm = state.clone();
        for _ in 0..150 {
            model.step_actuators(&mut warm, &input, wdt);
        }

        let dot = model.derivatives(&state, &input);
        dot.acceleration.norm_squared() + dot.angular_acceleration.norm_squared()
    };

    let (best, residual) = nelder_mead(&obj, &x0, &steps, &bounds, max_iter);

    if residual > tol {
        return Err(TrimError::NoConvergence {
            residual: residual.sqrt(),
            iters: max_iter,
        });
    }

    Ok(TrimResult {
        alpha_rad: best[2].to_radians(),
        throttle: best[0],
        elevator: best[1],
        residual: residual.sqrt(),
    })
}

/// Simple Nelder-Mead simplex optimisation for `n`-dimensional smooth objectives.
///
/// Minimises `f` starting from `x0` with per-dimension initial simplex steps.
/// All vertices are clamped to `bounds` after each update.
fn nelder_mead(
    f: &impl Fn(&[f64]) -> f64,
    x0: &[f64],
    steps: &[f64],
    bounds: &[(f64, f64)],
    max_iter: usize,
) -> (Vec<f64>, f64) {
    let n = x0.len();

    // Build initial simplex: x0 and x0 + steps[i]*e_i for each dimension
    let mut simplex: Vec<Vec<f64>> = Vec::with_capacity(n + 1);
    simplex.push(x0.to_vec());
    for i in 0..n {
        let mut v = x0.to_vec();
        v[i] += steps[i];
        clamp_vec(&mut v, bounds);
        simplex.push(v);
    }

    let mut values: Vec<f64> = simplex.iter().map(|v| f(v)).collect();

    for _ in 0..max_iter {
        // Sort simplex vertices by function value ascending
        let mut order: Vec<usize> = (0..=n).collect();
        order.sort_by(|&a, &b| values[a].partial_cmp(&values[b]).unwrap());

        simplex = order.iter().map(|&i| simplex[i].clone()).collect();
        values = order.iter().map(|&i| values[i]).collect();
        // Now simplex[0] = best, simplex[n] = worst

        // Centroid of best n vertices (exclude worst)
        let mut centroid = vec![0.0; n];
        for v in simplex.iter().take(n) {
            for (j, c) in centroid.iter_mut().enumerate() {
                *c += v[j];
            }
        }
        for c in &mut centroid {
            *c /= n as f64;
        }

        // Reflection
        let mut xr = vec![0.0; n];
        for j in 0..n {
            xr[j] = centroid[j] + (centroid[j] - simplex[n][j]);
        }
        clamp_vec(&mut xr, bounds);
        let fr = f(&xr);

        if fr < values[0] {
            // Try expansion
            let mut xe = vec![0.0; n];
            for j in 0..n {
                xe[j] = centroid[j] + 2.0 * (xr[j] - centroid[j]);
            }
            clamp_vec(&mut xe, bounds);
            let fe = f(&xe);
            if fe < fr {
                simplex[n] = xe;
                values[n] = fe;
            } else {
                simplex[n] = xr;
                values[n] = fr;
            }
        } else if fr < values[n - 1] {
            // Accept reflection
            simplex[n] = xr;
            values[n] = fr;
        } else {
            // Contraction
            let mut xc = vec![0.0; n];
            for j in 0..n {
                xc[j] = centroid[j] + 0.5 * (simplex[n][j] - centroid[j]);
            }
            clamp_vec(&mut xc, bounds);
            let fc = f(&xc);
            if fc < values[n] {
                simplex[n] = xc;
                values[n] = fc;
            } else {
                // Shrink all toward best
                let best = simplex[0].clone();
                for i in 1..=n {
                    for j in 0..n {
                        simplex[i][j] = best[j] + 0.5 * (simplex[i][j] - best[j]);
                    }
                    clamp_vec(&mut simplex[i], bounds);
                    values[i] = f(&simplex[i]);
                }
            }
        }
    }

    // Find best
    let mut best_idx = 0;
    for i in 1..=n {
        if values[i] < values[best_idx] {
            best_idx = i;
        }
    }

    (simplex[best_idx].clone(), values[best_idx])
}

/// Clamp each element of `v` to the corresponding `bounds`.
fn clamp_vec(v: &mut [f64], bounds: &[(f64, f64)]) {
    for (val, &(lo, hi)) in v.iter_mut().zip(bounds.iter()) {
        *val = val.clamp(lo, hi);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nelder_mead_finds_minimum_of_quadratic() {
        // f(x, y) = (x - 3)² + (y + 1)²  → minimum at (3, -1)
        let f = |x: &[f64]| (x[0] - 3.0).powi(2) + (x[1] + 1.0).powi(2);
        let bounds = vec![(-10.0, 10.0), (-10.0, 10.0)];
        let (best, val) = nelder_mead(&f, &[0.0, 0.0], &[1.0, 1.0], &bounds, 200);
        assert!(
            (best[0] - 3.0).abs() < 0.01 && (best[1] + 1.0).abs() < 0.01,
            "Expected (3, -1), got ({:.4}, {:.4})",
            best[0],
            best[1]
        );
        assert!(val < 1e-4, "Residual too high: {val:.6}");
    }

    #[test]
    fn trim_200ms_sea_level_converges() {
        let model = F16Model::f16a();
        let result = find_trim(&model, 200.0, 0.0);
        assert!(
            result.is_ok(),
            "Trim did not converge: {:?}",
            result.err()
        );
        let t = result.unwrap();
        // Alpha should be within the search bounds
        assert!(
            t.alpha_rad > -0.10 && t.alpha_rad < 0.44,
            "Alpha out of range: {:.2}°",
            t.alpha_rad.to_degrees()
        );
        // Throttle should be in (0.05, 1.0)
        assert!(
            t.throttle > 0.05 && t.throttle < 1.0,
            "Throttle out of range: {:.3}",
            t.throttle
        );
        println!(
            "Trim: α={:.2}°, throttle={:.3}, elevator={:.3}, residual={:.4}",
            t.alpha_rad.to_degrees(),
            t.throttle,
            t.elevator,
            t.residual
        );
    }
}
