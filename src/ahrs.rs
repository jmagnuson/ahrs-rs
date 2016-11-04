
use na::{Vector3, BaseFloat};

pub trait Ahrs<N: BaseFloat> {

  fn update( &mut self, gyroscope: Vector3<N>, accelerometer: Vector3<N>, magnetometer: Vector3<N> ) -> bool;
  fn update_imu( &mut self, gyroscope: Vector3<N>, accelerometer: Vector3<N> ) -> bool;
}

