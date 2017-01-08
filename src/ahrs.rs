
use na::{Vector3, BaseFloat, Quaternion};

/// Trait for implementing an AHRS filter.
pub trait Ahrs<N: BaseFloat> {

  /// Updates the current state quaternion using 9dof IMU values, made up by `gyroscope`,
  /// `accelerometer`, and `magnetometer`.
  fn update( &mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N>, magnetometer: &Vector3<N> ) -> Result<&Quaternion<N>, &str>;

  /// Updates the current state quaternion using 6dof IMU values, made up by `gyroscope` &
  /// `accelerometer`.
  fn update_imu( &mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N> ) -> Result<&Quaternion<N>, &str>;
}

