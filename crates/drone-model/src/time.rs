#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct TimeStep(f64);

impl TimeStep {
    pub fn new(dt: f64) -> Result<Self, TimeStepError> {
        if dt > 0.0 {
            Ok(Self(dt))
        } else {
            Err(TimeStepError(dt))
        }
    }

    pub fn constant(dt: f64) -> Self {
        Self::new(dt).expect("TimeStep::constant called with dt <= 0")
    }

    pub fn seconds(self) -> f64 {
        self.0
    }

    pub fn half(self) -> Self {
        Self(self.0 / 2.0)
    }
}

#[derive(Debug)]
pub struct TimeStepError(f64);

impl std::fmt::Display for TimeStepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TimeStep must be positive, got {}", self.0)
    }
}

impl std::error::Error for TimeStepError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn correct_dt() {
        let dt = TimeStep::new(0.01).unwrap();
        assert_eq!(dt.seconds(), 0.01);
    }

    #[test]
    fn zero_dt_is_error() {
        assert!(TimeStep::new(0.0).is_err());
    }

    #[test]
    fn negative_dt_is_error() {
        assert!(TimeStep::new(-0.01).is_err());
    }

    #[test]
    fn half_gives_half() {
        let dt = TimeStep::constant(0.01);
        assert_eq!(dt.half().seconds(), 0.005);
    }
}
