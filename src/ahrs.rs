
#![allow(non_snake_case)]
#![allow(dead_code)]

use na::{Vector2, Vector3, Vector4, Vector6, Matrix6, Matrix4, Norm, Quaternion, try_normalize};
use na;

#[derive(Debug, Clone, Copy)]
pub struct Ahrs {

  sample_period: f64,
  beta: f64,
  pub quat: Quaternion<f64>

}

impl Default for Ahrs {

  /// Creates a new Ahrs instance with default filter parameters.
  fn default() -> Ahrs {
    Ahrs { sample_period: (1.0f64)/(256.0f64),
           beta: 0.1f64,
           quat: Quaternion::new(1.0f64, 0.0, 0.0, 0.0)
         }
  }


}

impl Ahrs {

  /// Updates the current state quaternion using 9dof IMU values.
  pub fn update( &mut self, gyroscope: Vector3<f64>, accelerometer: Vector3<f64>, magnetometer: Vector3<f64> ) -> bool {
    let q = self.quat;
    
    // Nomralize accelerometer measurement
    let accel = match try_normalize(&accelerometer, 0.0f64) {
        Some(n) => n,
        None => { return false; }
    };
    
    // Nomralize magnetometer measurement
    let mag = match try_normalize(&magnetometer, 0.0f64) {
        Some(n) => n,
        None => { return false; }
    };

    // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
    let h = q * ( Quaternion::new(0.0f64, mag[0], mag[1], mag[2]) * self.quat );
    let b = Quaternion::new( 0.0f64, Norm::norm(&Vector2::new(h[1], h[2])), 0.0, h[3] );

    // Gradient descent algorithm corrective step
    let F = Vector6::new(
      2.0*(       q[1]*q[3] - q[0]*q[2]) - accel[0],
      2.0*(       q[0]*q[1] + q[2]*q[3]) - accel[1],
      2.0*( 0.5 - q[1]*q[1] - q[2]*q[2]) - accel[2],
      2.0*b[1]*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2.0*b[3]*(q[1]*q[3] - q[0]*q[2]) - mag[0],
      2.0*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2.0*b[3]*(      q[0]*q[1] + q[2]*q[3]) - mag[1],
      2.0*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2.0*b[3]*(0.5 - q[1]*q[1] - q[2]*q[2]) - mag[2] );
   
    let J_t = Matrix6::new(
      -2.0*q[2], 2.0*q[1],       0.0, -2.0*b[3]*q[2],                 -2.0*(b[1]*q[3]+b[3]*q[1]), 2.0*b[1]*q[2],
       2.0*q[3], 2.0*q[0], -4.0*q[1],  2.0*b[3]*q[3],                  2.0*(b[1]*q[2]+b[3]*q[0]), 2.0*(b[1]*q[3]-2.0*b[3]*q[1]),
      -2.0*q[0], 2.0*q[3], -4.0*q[2], -2.0*(2.0*b[1]*q[2]-b[3]*q[0]),  2.0*(b[1]*q[1]+b[3]*q[3]), 2.0*(b[1]*q[0]-2.0*b[3]*q[2]),
       2.0*q[1], 2.0*q[2],       0.0, -2.0*(2.0*b[1]*q[3]+b[3]*q[1]), -2.0*(b[1]*q[0]+b[3]*q[2]), 2.0*b[1]*q[1],
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
   
    let step:Vector6<f64> = na::normalize(&(J_t * F));

    // Compute rate of change for quaternion
    let qDot:Quaternion<f64> = q * Quaternion::new(0.0f64, gyroscope[0], gyroscope[1], gyroscope[2])
        * 0.5 - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

    // Integrate to yield quaternion
    self.quat = na::normalize( &(q + qDot * self.sample_period) );

    true
  }

  /// Updates the current state quaternion using 6dof IMU values.
  pub fn update_imu( &mut self, gyroscope: Vector3<f64>, accelerometer: Vector3<f64> ) -> bool {
    let q = self.quat;

    // Normalize accelerometer measurement
    let accel = match try_normalize(&accelerometer, 0.0f64) {
      Some(n) => n,
      None => { return false; }
    };

    // Gradient descent algorithm corrective step
    let F = Vector4::new( 2.0*(      q[1]*q[3] - q[0]*q[2]) - accel[0],
                          2.0*(      q[0]*q[1] + q[2]*q[3]) - accel[1],
                          2.0*(0.5 - q[1]*q[1] - q[2]*q[2]) - accel[2],
                          0.0 );

    let J_t = Matrix4::new( -2.0*q[2], 2.0*q[1],            0.0, 0.0,
                             2.0*q[3], 2.0*q[0], -4.0*q[1], 0.0,
                            -2.0*q[0], 2.0*q[3], -4.0*q[2], 0.0,
                             2.0*q[1], 2.0*q[2],            0.0, 0.0 );

    let step: Vector4<f64> = na::normalize(&(J_t * F));

    // Compute rate of change of quaternion
    let qDot:Quaternion<f64> = (q * Quaternion::new(0.0f64, gyroscope[0], gyroscope[1], gyroscope[2]))
        * 0.5 - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

    // Integrate to yield quaternion
    self.quat = na::normalize( &(q + qDot * self.sample_period) );

    true
  }

}

