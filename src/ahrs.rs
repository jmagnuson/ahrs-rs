use nalgebra::{Quaternion, Scalar, Vector3};
use simba::simd::SimdValue;
use thiserror::Error;

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
    ) -> Result<&Quaternion<N>, AhrsError>;

    /// Attempts to update the current state quaternion using 6dof IMU values, made up by `gyroscope` &
    /// `accelerometer`.
    ///
    /// Returns a reference to the updated quaternion on success, or in the case of failure, an
    /// `Err(&str)` containing the reason.
    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> Result<&Quaternion<N>, AhrsError>;
}

#[derive(Error, Debug)]
pub enum AhrsError {
    #[error("Division by zero")]
    DivByZero,
}
