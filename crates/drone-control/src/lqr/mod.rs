pub mod care;
pub mod linearize;
pub mod lqi;
pub mod lqr;

// CareError and LqiError are the primary error types callers should match on.
pub use care::{CareError, RiccatiSolution, SolverParams};
pub use linearize::{LinearizedModel, linearize};
pub use lqi::{LqiController, LqiError, quadrotor_c_integral};
pub use lqr::LqrController;
// build_q_diagonal / build_r_diagonal are internal helpers; access via
// `care::build_q_diagonal` if you really need them from outside the crate.
