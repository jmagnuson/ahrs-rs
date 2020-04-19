
use crate::na::{Vector3, Quaternion};
use alga::general::RealField;

/// Trait for implementing an AHRS filter.
pub trait Ahrs<N: RealField> {

  /// Attempts to update the current state quaternion using 9dof IMU values, made up by `gyroscope`,
  /// `accelerometer`, and `magnetometer`.
  ///
  /// Returns a reference to the updated quaternion on success, or in the case of failure, an
  /// `Err(&str)` containing the reason.
  fn update( &mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N>, magnetometer: &Vector3<N> ) -> Result<&Quaternion<N>, &str>;

  /// Attempts to update the current state quaternion using 6dof IMU values, made up by `gyroscope` &
  /// `accelerometer`.
  ///
  /// Returns a reference to the updated quaternion on success, or in the case of failure, an
  /// `Err(&str)` containing the reason.
  fn update_imu( &mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N> ) -> Result<&Quaternion<N>, &str>;
}

