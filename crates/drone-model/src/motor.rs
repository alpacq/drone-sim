use std::ops::{Index, IndexMut};

///   FrontLeft(CCW)  FrontRight(CW)
///         \              /
///          \            /
///           [  BODY  ]
///          /            \
///         /              \
///   RearLeft(CW)    RearRight(CCW)

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Motor {
    FrontRight = 0,
    FrontLeft = 1,
    RearLeft = 2,
    RearRight = 3,
}

impl Motor {
    pub const ALL: [Motor; 4] = [
        Motor::FrontRight,
        Motor::FrontLeft,
        Motor::RearLeft,
        Motor::RearRight,
    ];

    pub fn is_clockwise(self) -> bool {
        matches!(self, Motor::FrontRight | Motor::RearLeft)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorArray<T>([T; 4]);

impl<T> MotorArray<T> {
    pub fn new(front_right: T, front_left: T, rear_left: T, rear_right: T) -> Self {
        Self([front_right, front_left, rear_left, rear_right])
    }

    pub fn iter(&self) -> impl Iterator<Item = (Motor, &T)> {
        Motor::ALL.iter().map(|&m| (m, &self[m]))
    }
}

impl<T: Copy> MotorArray<T> {
    pub fn uniform(value: T) -> Self {
        Self([value, value, value, value])
    }

    pub fn map<U, F: Fn(T) -> U>(&self, f: F) -> MotorArray<U> {
        MotorArray([
            f(self[Motor::FrontRight]),
            f(self[Motor::FrontLeft]),
            f(self[Motor::RearLeft]),
            f(self[Motor::RearRight]),
        ])
    }

    pub fn map_with_motor<U, F>(&self, f: F) -> MotorArray<U>
    where
        F: Fn(Motor, T) -> U,
    {
        MotorArray::new(
            f(Motor::FrontRight, self[Motor::FrontRight]),
            f(Motor::FrontLeft, self[Motor::FrontLeft]),
            f(Motor::RearLeft, self[Motor::RearLeft]),
            f(Motor::RearRight, self[Motor::RearRight]),
        )
    }

    pub fn sum(self) -> T
    where
        T: std::ops::Add<Output = T>,
    {
        let [a, b, c, d] = self.0;
        ((a + b) + c) + d
    }
}

impl<T> Index<Motor> for MotorArray<T> {
    type Output = T;

    fn index(&self, motor: Motor) -> &T {
        &self.0[motor as usize]
    }
}

impl<T> IndexMut<Motor> for MotorArray<T> {
    fn index_mut(&mut self, motor: Motor) -> &mut T {
        &mut self.0[motor as usize]
    }
}

impl<T> From<[T; 4]> for MotorArray<T> {
    fn from(arr: [T; 4]) -> Self {
        Self(arr)
    }
}

impl<T> From<MotorArray<T>> for [T; 4] {
    fn from(ma: MotorArray<T>) -> Self {
        ma.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexing_by_motor() {
        let arr = MotorArray::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(arr[Motor::FrontRight], 1.0);
        assert_eq!(arr[Motor::FrontLeft], 2.0);
        assert_eq!(arr[Motor::RearLeft], 3.0);
        assert_eq!(arr[Motor::RearRight], 4.0);
    }

    #[test]
    fn uniform_gives_the_same_value() {
        let arr = MotorArray::uniform(42.0_f64);
        for m in Motor::ALL {
            assert_eq!(arr[m], 42.0);
        }
    }

    #[test]
    fn map_applies_function() {
        let arr = MotorArray::uniform(2.0_f64);
        let squared = arr.map(|x| x * x);
        for m in Motor::ALL {
            assert_eq!(squared[m], 4.0);
        }
    }

    #[test]
    fn rotations_are_different() {
        // CW i CCW has to balance each other due to yaw
        let cw_count = Motor::ALL.iter().filter(|m| m.is_clockwise()).count();
        let ccw_count = Motor::ALL.iter().filter(|m| !m.is_clockwise()).count();
        assert_eq!(cw_count, 2);
        assert_eq!(ccw_count, 2);
    }
}
