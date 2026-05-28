pub fn interp1d(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    debug_assert_eq!(xs.len(), ys.len());
    debug_assert!(xs.len() >= 2);

    // Clamp to table range (also handles NaN — all comparisons with NaN are
    // false, so a NaN would fall through to `partition_point` and cause an
    // underflow panic.  Returning ys[0] for non-finite inputs is safe because
    // the aero model already clamps α and β to finite intervals before calling
    // this function; a NaN here indicates an upstream bug, not a valid state).
    if !x.is_finite() || x <= xs[0] {
        return ys[0];
    }
    if x >= xs[xs.len() - 1] {
        return ys[ys.len() - 1];
    }

    // Find interval through binary search
    let idx = xs.partition_point(|&xi| xi < x) - 1;

    // Linear interpolation: y = y0 + (y1-y0) * (x-x0)/(x1-x0)
    let t = (x - xs[idx]) / (xs[idx + 1] - xs[idx]);
    ys[idx] + t * (ys[idx + 1] - ys[idx])
}

pub fn interp2d(xs: &[f64], ys: &[f64], data: &[f64], x: f64, y: f64) -> f64 {
    debug_assert_eq!(xs.len() * ys.len(), data.len());

    let nx = xs.len();
    let ny = ys.len();

    // Clamp and find indices
    let ix = if x <= xs[0] {
        0
    } else if x >= xs[nx - 1] {
        nx - 2
    } else {
        xs.partition_point(|&xi| xi < x) - 1
    };

    let iy = if y <= ys[0] {
        0
    } else if y >= ys[ny - 1] {
        ny - 2
    } else {
        ys.partition_point(|&yi| yi < y) - 1
    };

    // Interpolation parameters
    let tx = if xs[ix + 1] != xs[ix] {
        (x - xs[ix]) / (xs[ix + 1] - xs[ix])
    } else {
        0.0
    };

    let ty = if ys[iy + 1] != ys[iy] {
        (y - ys[iy]) / (ys[iy + 1] - ys[iy])
    } else {
        0.0
    };

    // Four corners of the cell
    let f00 = data[ix * ny + iy];
    let f10 = data[(ix + 1) * ny + iy];
    let f01 = data[ix * ny + iy + 1];
    let f11 = data[(ix + 1) * ny + iy + 1];

    // Bilinear interpolation
    f00 * (1.0 - tx) * (1.0 - ty) + f10 * tx * (1.0 - ty) + f01 * (1.0 - tx) * ty + f11 * tx * ty
}

// ── Tables axises ────────────────────────────────────────────────────

/// Values α [degrees] — axis for most tables.
/// Range: -10° to +45°, step 5°.
pub const ALPHA_BREAK: &[f64] = &[
    -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0,
];

/// Values β [degrees] — axis for side tables.
/// Range: -30° to +30°, step 5°.
pub const BETA_BREAK: &[f64] = &[
    -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0,
];

/// Deflections of ailerons [degrees] for roll control tables.
pub const AILERON_BREAK: &[f64] = &[-21.5, -13.0, -6.5, 0.0, 6.5, 13.0, 21.5];

/// Deflections of rudder [degrees] for yaw control tables.
pub const RUDDER_BREAK: &[f64] = &[-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0];

// ── Tables of forces and moments ─────────────────────────────────────────

/// CX(α) — coefficient of longitudinal force (x_body axis, + = forward thrust, - = rear thrust).
///
/// In aerodynamic convention: CX = -CD·cos(α) + CL·sin(α)
/// Negative = rear thrust.
pub const CX_ALPHA: &[f64] = &[
    -0.099, -0.081, -0.081, -0.063, -0.025, 0.044, 0.097, 0.113, 0.145, 0.167, 0.174, 0.166,
];

/// CZ(α) — coefficient of normal force (z_body axis, + = upward in body NED = downward in ENU).
///
/// WARNING: In NASA TP-1538, the z_body axis is directed downward (NED).
/// CZ negative = downwards force (as seen from the front).
/// Converting the sign using: F_z_ENU = -CZ · q_bar · S
pub const CZ_ALPHA: &[f64] = &[
    0.770, 0.241, -0.100, -0.416, -0.731, -1.053, -1.366, -1.646, -1.917, -2.120, -2.248, -2.229,
];

/// CM(α) — base pitch moment (without the effect of altitude and trim).
///
/// Negative CM_α = longitudinal stability (moment restoring when α increases).
/// F-16 has natural relaxed static stability (RSS) — requires fly-by-wire to stabilize.
pub const CM_ALPHA: &[f64] = &[
    0.205, 0.168, 0.186, 0.196, 0.213, 0.251, 0.245, 0.238, 0.252, 0.231, 0.198, 0.192,
];

/// CY(β) — coefficient of lateral force.
/// Linear in β for small glide angles.
/// Value for β = -30°..+30°.
pub const CY_BETA: &[f64] = &[
    -0.232, -0.196, -0.160, -0.124, -0.088, -0.044, 0.000, 0.044, 0.088, 0.124, 0.160, 0.196, 0.232,
];

/// CL base (β) — moment roll from glide angle (sweep effect).
pub const CL_BETA: &[f64] = &[
    -0.0126, -0.0105, -0.0084, -0.0063, -0.0042, -0.0021, 0.0000, 0.0021, 0.0042, 0.0063, 0.0084,
    0.0105, 0.0126,
];

/// CN base (β) — moment yaw from glide angle (yaw stability effect).
pub const CN_BETA: &[f64] = &[
    -0.0437, -0.0364, -0.0292, -0.0219, -0.0146, -0.0073, 0.0000, 0.0073, 0.0146, 0.0219, 0.0292,
    0.0364, 0.0437,
];

// ── Control derivatives ───────────────────────────────────────────

/// CZ_δe — effectiveness of altitude control (change CZ by 1 degree δe).
/// Constant in subsonic range.
pub const CZ_DE_PER_DEG: f64 = -0.19;

/// CM_δe — effectiveness of pitch moment from altitude control [1/°].
pub const CM_DE_PER_DEG: f64 = -0.05;

/// CL_δa(α) — effectiveness of lift from angle of attack (AoA) [1/°].
/// Decreases with increasing α (loss of roll control at high AoA).
pub const CL_DA_ALPHA: &[f64] = &[
    -0.041, -0.041, -0.042, -0.040, -0.043, -0.044, -0.043, -0.037, -0.030, -0.020, -0.010, -0.004,
];

/// CN_δa(α) — effectiveness of yaw moment from angle of attack (AoA) [1/°].
/// Unfavorable yaw effect at high AoA.
pub const CN_DA_ALPHA: &[f64] = &[
    0.005, 0.005, 0.005, 0.005, 0.005, 0.009, 0.011, 0.010, 0.008, 0.005, 0.003, 0.002,
];

/// CL_δr — effectiveness of roll control (change CL by 1 degree δr).
pub const CL_DR_PER_DEG: f64 = 0.005;

/// CN_δr — effectiveness of yaw control (change CN by 1 degree δr).
pub const CN_DR_PER_DEG: f64 = -0.022;

// ── Damping derivatives ─────────────────────────────────────────────

/// CX_q(α) — effectiveness of longitudinal damping from pitch speed [1/rad].
pub const CXQ_ALPHA: &[f64] = &[
    -0.267, -0.110, 0.308, 1.340, 2.080, 2.910, 2.760, 2.050, 1.500, 1.490, 1.830, 1.210,
];

/// CZ_q(α) — effectiveness of normal damping from pitch speed [1/rad].
pub const CZQ_ALPHA: &[f64] = &[
    -8.800, -25.80, -28.90, -31.40, -31.20, -30.70, -27.70, -28.20, -29.00, -29.80, -38.30, -35.30,
];

/// CM_q(α) — effectiveness of pitch moment from pitch speed [1/rad].
/// Negative = stabilizes pitch oscillations (key for flying).
pub const CMQ_ALPHA: &[f64] = &[
    -8.800, -25.80, -28.90, -31.40, -31.20, -30.70, -27.70, -28.20, -29.00, -29.80, -38.30, -35.30,
];

/// CL_p(α) — effectiveness of roll damping from pitch speed [1/rad].
/// Negative = stabilizes roll oscillations (key for flying).
pub const CLP_ALPHA: &[f64] = &[
    -0.410, -0.410, -0.416, -0.416, -0.454, -0.466, -0.486, -0.499, -0.514, -0.529, -0.544, -0.559,
];

/// CN_r(α) — effectiveness of yaw damping from yaw speed [1/rad].
/// Negative = stabilizes yaw oscillations (key for flying).
pub const CNR_ALPHA: &[f64] = &[
    -0.126, -0.126, -0.128, -0.128, -0.140, -0.143, -0.149, -0.153, -0.157, -0.162, -0.166, -0.171,
];

/// CL_r(α) — feedback yaw→roll [1/rad].
pub const CLR_ALPHA: &[f64] = &[
    0.250, 0.250, 0.260, 0.260, 0.280, 0.290, 0.305, 0.315, 0.330, 0.340, 0.358, 0.368,
];

/// CN_p(α) — feedback roll→yaw [1/rad].
pub const CNP_ALPHA: &[f64] = &[
    0.061, 0.061, 0.069, 0.069, 0.082, 0.090, 0.101, 0.113, 0.123, 0.133, 0.142, 0.152,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interp1d_in_range() {
        let xs = &[0.0, 1.0, 2.0];
        let ys = &[0.0, 1.0, 4.0];
        // Exact points
        assert!((interp1d(xs, ys, 0.0) - 0.0).abs() < 1e-10);
        assert!((interp1d(xs, ys, 1.0) - 1.0).abs() < 1e-10);
        assert!((interp1d(xs, ys, 2.0) - 4.0).abs() < 1e-10);
        // Midpoint
        assert!((interp1d(xs, ys, 0.5) - 0.5).abs() < 1e-10);
        assert!((interp1d(xs, ys, 1.5) - 2.5).abs() < 1e-10);
    }

    #[test]
    fn interp1d_clamp_out_of_range() {
        let xs = &[0.0, 1.0];
        let ys = &[0.0, 1.0];
        assert_eq!(interp1d(xs, ys, -1.0), 0.0); // clamp do lewej
        assert_eq!(interp1d(xs, ys, 2.0), 1.0); // clamp do prawej
    }

    #[test]
    fn interp2d_on_grid() {
        // f(x,y) = x + y
        let xs = &[0.0, 1.0];
        let ys = &[0.0, 1.0];
        let data = &[
            0.0, 1.0, // f(0,0)=0, f(0,1)=1
            1.0, 2.0, // f(1,0)=1, f(1,1)=2
        ];
        assert!((interp2d(xs, ys, data, 0.5, 0.5) - 1.0).abs() < 1e-10);
        assert!((interp2d(xs, ys, data, 0.0, 0.0) - 0.0).abs() < 1e-10);
        assert!((interp2d(xs, ys, data, 1.0, 1.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn tables_have_correct_length() {
        assert_eq!(CX_ALPHA.len(), ALPHA_BREAK.len());
        assert_eq!(CZ_ALPHA.len(), ALPHA_BREAK.len());
        assert_eq!(CM_ALPHA.len(), ALPHA_BREAK.len());
        assert_eq!(CY_BETA.len(), BETA_BREAK.len());
        assert_eq!(CL_BETA.len(), BETA_BREAK.len());
        assert_eq!(CN_BETA.len(), BETA_BREAK.len());
    }

    #[test]
    fn cx_increases_with_alpha() {
        // CX should increase with α (less drag at higher α)
        let alpha_low = interp1d(ALPHA_BREAK, CX_ALPHA, -10.0);
        let alpha_high = interp1d(ALPHA_BREAK, CX_ALPHA, 20.0);
        assert!(
            alpha_high > alpha_low,
            "CX should increase with α: low={:.3}, high={:.3}",
            alpha_low,
            alpha_high
        );
    }

    #[test]
    fn cz_negative_for_normal_flight() {
        // CZ negative = lift (up) in NED convention
        let cz = interp1d(ALPHA_BREAK, CZ_ALPHA, 5.0);
        assert!(cz < 0.0, "CZ should be negative (lift) at α=5°: {:.3}", cz);
    }

    #[test]
    fn cy_antysymmetric_about_beta() {
        // CY should be antysymmetric: CY(-β) = -CY(β)
        let cy_pos = interp1d(BETA_BREAK, CY_BETA, 10.0);
        let cy_neg = interp1d(BETA_BREAK, CY_BETA, -10.0);
        assert!(
            (cy_pos + cy_neg).abs() < 1e-10,
            "CY should be antisymmetric: CY(10°)={:.4}, CY(-10°)={:.4}",
            cy_pos,
            cy_neg
        );
    }
}
