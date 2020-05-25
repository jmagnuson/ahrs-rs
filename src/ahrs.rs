use displaydoc::Display;
use nalgebra::{Scalar, UnitQuaternion, Vector3};
use simba::simd::SimdValue;

/// Trait for implementing an AHRS filter.
pub trait Ahrs<N: Scalar + SimdValue> {
    /// Attempts to update the current state quaternion using 9dof IMU values, made up by `gyroscope`,
    /// `accelerometer`, and `magnetometer`.
    ///
    /// Returns a reference to the updated quaternion on success, or in the case of failure, an
    /// `Err(&str)` containing the reason.
    fn update(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
        magnetometer: &Vector3<N>,
    ) -> Result<&UnitQuaternion<N>, AhrsError>;

    /// Attempts to update the current state quaternion using 6dof IMU values, made up by `gyroscope` &
    /// `accelerometer`.
    ///
    /// Returns a reference to the updated quaternion on success, or in the case of failure, an
    /// `Err(&str)` containing the reason.
    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> Result<&UnitQuaternion<N>, AhrsError>;
}

#[derive(Display, Debug)]
pub enum AhrsError {
    /// Division by zero
    DivByZero,
}

#[cfg(feature = "std")]
impl std::error::Error for AhrsError {}

#[cfg(test)]
mod tests {
    use super::AhrsError;

    #[cfg(feature = "std")]
    #[test]
    fn test_ahrserror_is_error() {
        fn is_error<T: std::error::Error>() {}
        is_error::<AhrsError>();
    }

    #[test]
    fn test_ahrserror_is_display() {
        fn is_display<T: core::fmt::Display>() {}
        is_display::<AhrsError>();
    }
}
