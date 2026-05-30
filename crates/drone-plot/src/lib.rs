pub mod comparison;
pub mod monte_carlo;
pub mod scenario;
pub mod validation;

mod palette;

pub use comparison::plot_comparison;
pub use monte_carlo::plot_monte_carlo;
pub use scenario::plot_scenario;
pub use validation::plot_validation;

/// Zamień dowolną nazwę na bezpieczny fragment nazwy pliku.
pub(crate) fn slug(s: &str) -> String {
    s.chars()
        .map(|c| if c.is_alphanumeric() || c == '-' { c } else { '_' })
        .collect()
}
