#![allow(non_snake_case)]

use na::{Vector2, Vector3, BaseFloat, Quaternion, try_normalize, Norm};
use na;
use ahrs::Ahrs;


#[derive(Eq, PartialEq, Clone, Debug, Hash, Copy)]
pub struct Mahony<N: BaseFloat> {

    sample_period: N,
    pub quat: Quaternion<N>,
    kp: N,
    ki: N,
    e_int: Vector3<N>

}

impl Default for Mahony<f64> {

    fn default() -> Mahony<f64> {
        Mahony { sample_period: (1.0f64)/(256.0),
                  quat: Quaternion::new(1.0f64, 0.0, 0.0, 0.0),
                  kp: 0.5f64,
                  ki: 0.0f64,
                  e_int: Vector3::new(0.0, 0.0, 0.0)
        }
    }
}

impl<N: BaseFloat> Ahrs<N> for Mahony<N> {

  fn update( &mut self, gyroscope: Vector3<N>, accelerometer: Vector3<N>, magnetometer: Vector3<N> ) -> bool {

    let q = self.quat;
    
    let zero: N = na::zero();
    let two: N = na::cast(2.0);
    let half: N = na::cast(0.5);

    // Nomralize accelerometer measurement
    let accel = match try_normalize(&accelerometer, zero) {
        Some(n) => n,
        None => { return false; }
    };
    
    // Nomralize magnetometer measurement
    let mag = match try_normalize(&magnetometer, zero) {
        Some(n) => n,
        None => { return false; }
    };

    // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
    let h = q * ( Quaternion::from_parts(zero, mag) * q.conjugate() );
    let b = Quaternion::new( zero, Norm::norm(&Vector2::new(h[1], h[2])), zero, h[3] );
 
    let v = Vector3::new(
      two*( q[1]*q[3] - q[0]*q[2] ),
      two*( q[0]*q[1] + q[2]*q[3] ),
      q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3] );

    let w = Vector3::new(
        two*b[1]*(half - q[2]*q[2] - q[3]*q[3]) + two*b[3]*(q[1]*q[3] - q[0]*q[2]),
        two*b[1]*(q[1]*q[2] - q[0]*q[3]) + two*b[3]*(q[0]*q[1] + q[2]*q[3]),
        two*b[1]*(q[0]*q[2] + q[1]*q[3]) + two*b[3]*(half - q[1]*q[1] - q[2]*q[2]));

    let e: Vector3<N> = na::cross(&accel, &v) + na::cross(&mag, &w);
    
    // Error is sum of cross product between estimated direction and measured direction of fields
    if self.ki > zero  {
        self.e_int += e * self.sample_period;
    } else {
        //Vector3::new(zero, zero, zero);
        self.e_int.x = zero; self.e_int.y = zero; self.e_int.z = zero;
    }
    
    // Apply feedback terms
    let gyro = gyroscope + e * self.kp + self.e_int * self.ki;

    // Compute rate of change of quaternion
    let qDot = q * Quaternion::from_parts(zero, gyro) * half;

    // Integrate to yield quaternion
    self.quat = na::normalize( &(q + qDot * self.sample_period) );

    true
  }

  fn update_imu( &mut self, gyroscope: Vector3<N>, accelerometer: Vector3<N> ) -> bool {

    let q = self.quat;
    
    let zero: N = na::zero();
    let two: N = na::cast(2.0);
    let half: N = na::cast(0.5);

    // Nomralize accelerometer measurement
    let accel = match try_normalize(&accelerometer, zero) {
        Some(n) => n,
        None => { return false; }
    };

    let v = Vector3::new(
      two*( q[1]*q[3] - q[0]*q[2] ),
      two*( q[0]*q[1] + q[2]*q[3] ),
      q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3] );

    let e = na::cross(&accel, &v);

    // Error is sum of cross product between estimated direction and measured direction of fields
    if self.ki > zero {
        self.e_int += e * self.sample_period;
//        self.e_int += e;
    } else {
        self.e_int.x = zero; self.e_int.y = zero; self.e_int.z = zero;
    }

    // Apply feedback terms
    let gyro = gyroscope + e * self.kp + self.e_int * self.ki;

    // Compute rate of change of quaternion
    let qDot = q * Quaternion::from_parts(zero, gyro) * half;

    // Integrate to yield quaternion
    self.quat = na::normalize( &(q + qDot * self.sample_period) );

    true
  }

}

