use anyhow::Result;
use nalgebra::DMatrix;

#[derive(Debug)]
pub struct StabilityAnalysis {
    pub is_stable: bool,
    pub closed_loop_poles: Vec<(f64, f64)>,
    pub dominant_pole: (f64, f64),
    pub controllability_rank: usize,
    pub system_order: usize,
}

impl StabilityAnalysis {
    pub fn is_controllable(&self) -> bool {
        self.controllability_rank == self.system_order
    }
}

pub fn analyze_stability(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    k: &DMatrix<f64>,
) -> Result<StabilityAnalysis> {
    let n = a.nrows();
    anyhow::ensure!(k.ncols() == n, "K dimensions not matched with A");

    let a_cl = a - b * k;

    let eigenvalues = compute_eigenvalues_approx(&a_cl);

    let is_stable = eigenvalues.iter().all(|(re, _)| *re < 0.0);

    let dominant = eigenvalues
        .iter()
        .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
        .copied()
        .unwrap_or((0.0, 0.0));

    let ctrl_rank = controllability_rank(a, b);

    Ok(StabilityAnalysis {
        is_stable,
        closed_loop_poles: eigenvalues,
        dominant_pole: dominant,
        controllability_rank: ctrl_rank,
        system_order: n,
    })
}

fn compute_eigenvalues_approx(a: &DMatrix<f64>) -> Vec<(f64, f64)> {
    let n = a.nrows();

    if n == 1 {
        return vec![(a[(0, 0)], 0.0)];
    }

    if n == 2 {
        let tr = a[(0, 0)] + a[(1, 1)];
        let det = a[(0, 0)] * a[(1, 1)] - a[(0, 1)] * a[(1, 0)];
        let disc = tr * tr - 4.0 * det;

        if disc >= 0.0 {
            let s = disc.sqrt();
            return vec![((tr + s) / 2.0, 0.0), ((tr - s) / 2.0, 0.0)];
        } else {
            let re = tr / 2.0;
            let im = (-disc).sqrt() / 2.0;
            return vec![(re, im), (re, -im)];
        }
    }

    let trace = (0..n).map(|i| a[(i, i)]).sum::<f64>();

    (0..n)
        .map(|i| {
            let diag = a[(i, i)];
            // Przybliżenie: wartość własna ≈ element diagonalny
            // dla macierzy dominujących diagonalnie
            (diag, 0.0)
        })
        .collect()
}

pub fn controllability_rank(a: &DMatrix<f64>, b: &DMatrix<f64>) -> usize {
    let n = a.nrows();
    let m = b.ncols();
    let mut kalman = DMatrix::zeros(n, n * m);

    let mut ab = b.clone();
    for i in 0..n {
        let start_col = i * m;
        for col in 0..m {
            kalman.set_column(start_col + col, &ab.column(col));
        }
        ab = a * &ab;
    }

    let svd = kalman.svd(false, false);
    let threshold = 1e-10 * svd.singular_values[0];
    svd.singular_values
        .iter()
        .filter(|&&s| s > threshold)
        .count()
        .min(n)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_1d_system() {
        // ẋ = -2x + u, K = [1] → A_cl = -3
        let a = DMatrix::from_element(1, 1, -2.0);
        let b = DMatrix::from_element(1, 1, 1.0);
        let k = DMatrix::from_element(1, 1, 1.0);
        let r = analyze_stability(&a, &b, &k).unwrap();
        assert!(r.is_stable, "Układ powinien być stabilny");
    }

    #[test]
    fn unstable_1d_system() {
        // ẋ = 2x, K = 0 → A_cl = 2 (unstable)
        let a = DMatrix::from_element(1, 1, 2.0);
        let b = DMatrix::from_element(1, 1, 1.0);
        let k = DMatrix::from_element(1, 1, 0.0);
        let r = analyze_stability(&a, &b, &k).unwrap();
        assert!(!r.is_stable, "Układ powinien być niestabilny");
    }

    #[test]
    fn controllability_rank_double_integrator() {
        // ẋ₁ = x₂, ẋ₂ = u → fully controllable
        let a = DMatrix::from_row_slice(2, 2, &[0.0, 1.0, 0.0, 0.0]);
        let b = DMatrix::from_row_slice(2, 1, &[0.0, 1.0]);
        assert_eq!(controllability_rank(&a, &b), 2);
    }

    #[test]
    fn uncontrollable_system() {
        // x₂ is not under control → rank = 1
        let a = DMatrix::from_row_slice(2, 2, &[0.0, 0.0, 0.0, 0.0]);
        let b = DMatrix::from_row_slice(2, 1, &[1.0, 0.0]);
        assert_eq!(controllability_rank(&a, &b), 1);
    }
}
