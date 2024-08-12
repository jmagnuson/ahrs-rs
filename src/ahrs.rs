use nalgebra::{Scalar, UnitQuaternion, Vector3};
use simba::simd::SimdValue;

#[derive(Debug)]
pub enum AhrsError {
    AccelerometerNormZero,
    MagnetometerNormZero,
}

/// Trait for implementing an AHRS filter.
pub trait Ahrs<N: Scalar + SimdValue> {
    /// Attempts to update the current state quaternion using 9dof IMU values, made up by `gyroscope`,
    /// `accelerometer`, and `magnetometer`.
    ///
    /// Returns a reference to the updated quaternion on success, or in the case of failure, an
    /// `AhrsError` enum, which describes the reason.
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
    /// `AhrsError` enum, which describes the reason.
    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> Result<&UnitQuaternion<N>, AhrsError>;

    /// Updates the current state quaternion using only 3dof IMU values, made up by `gyroscope`.
    ///
    /// Returns a reference to the updated quaternion.
    fn update_gyro(
        &mut self,
        gyroscope: &Vector3<N>,
    ) -> &UnitQuaternion<N>;
}
