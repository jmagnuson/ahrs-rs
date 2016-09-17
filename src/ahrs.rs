
#![allow(non_snake_case)]
#![allow(dead_code)]

use na::{Vector2, Vector3, Vector6, Matrix6, Norm, Cross, Quaternion};
use na;

#[derive(Debug, Clone, Copy)]
pub struct AHRS {

  sample_period: f64,
  pub quat: Quaternion<f64>,
  kp: f64,
  ki: f64,
  kp_init: f64,
  init_period: f64,

  q: Quaternion<f64>,
  int_error: Vector3<f64>,
  kp_ramped: f64,

  // temporary until update() is written properly
  beta: f64
}

impl AHRS {

    /// Creates a new AHRS instance with default filter parameters.
  pub fn default() -> AHRS {
    AHRS { sample_period: (1.0f64)/(256.0f64), quat: Quaternion::new(1.0f64, 0.0, 0.0, 0.0),
           kp: 1.0f64, ki: 0.0f64, kp_init: 1.0, init_period: 5.0f64, q: Quaternion::new(1.0f64, 0.0, 0.0, 0.0),
           int_error: Vector3::new(0.0f64, 0.0, 0.0), kp_ramped: 0.0f64, beta: 1.0f64
         }
  }

  /// Updates the current state quaternion using 9dof IMU values.
  pub fn update( &mut self, gyroscope: Vector3<f64>, accelerometer: Vector3<f64>, magnetometer: Vector3<f64> ) -> bool {
    
    // Nomralize accelerometer measurement
    if Norm::norm(&accelerometer) > 0.0f64 {
      na::normalize( &accelerometer );
    } else {
      return false;
    }
    
    // Nomralize magnetometer measurement
    if Norm::norm(&magnetometer) > 0.0f64 {
      na::normalize( &magnetometer );
    } else {
      return false;
    }

    // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
    let h = self.q * ( Quaternion::new(0.0f64, magnetometer[0], magnetometer[1], magnetometer[2]) * self.quat );
    let b = Quaternion::new( 0.0f64, Norm::norm(&Vector2::new(h[2], h[3])), 0.0, h[4] );

    // Gradient descent algorithm corrective step
    let F = Vector6::new(
      2.0*(self.q[1]*self.q[3] - self.q[0]*self.q[2]) - accelerometer[0],
      2.0*(self.q[0]*self.q[1] + self.q[2]*self.q[3]) - accelerometer[1],
      2.0*(0.5 - self.q[1]*self.q[1] - self.q[2]*self.q[2]) - accelerometer[2],
      2.0*b[1]*(0.5 - self.q[2]*self.q[2] - self.q[3]*self.q[3]) + 2.0*b[3]*(self.q[1]*self.q[3] - self.q[0]*self.q[2]) - magnetometer[0],
      2.0*b[1]*(self.q[1]*self.q[2] - self.q[0]*self.q[3]) + 2.0*b[3]*(self.q[0]*self.q[1] + self.q[2]*self.q[3]) - magnetometer[1],
      2.0*b[1]*(self.q[0]*self.q[2] + self.q[1]*self.q[3]) + 2.0*b[3]*(0.5 - self.q[1]*self.q[1] - self.q[2]*self.q[2]) - magnetometer[2] );
   
    let J_t = Matrix6::new(
      -2.0*self.q[2], 2.0*self.q[1], 0.0, -2.0*b[3]*self.q[2], -2.0*(b[1]*self.q[3]+b[3]*self.q[1]), 2.0*b[1]*self.q[2],
      2.0*self.q[3], 2.0*self.q[0], -4.0*self.q[1], 2.0*b[3]*self.q[3], 2.0*(b[1]*self.q[2]+b[3]*self.q[0]), 2.0*(b[1]*self.q[3]-2.0*b[3]*self.q[1]),
      -2.0*self.q[0], 2.0*self.q[3], -4.0*self.q[2], -2.0*(2.0*b[1]*self.q[2]-b[3]*self.q[0]), 2.0*(b[1]*self.q[1]+b[3]*self.q[3]), 2.0*(b[1]*self.q[0]-2.0*b[3]*self.q[2]),
      2.0*self.q[1], 2.0*self.q[2], 0.0, -2.0*(2.0*b[1]*self.q[3]+b[3]*self.q[1]), -2.0*(b[1]*self.q[0]+b[3]*self.q[2]), 2.0*b[1]*self.q[1],
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
   
    let step:Vector6<f64> = J_t * F;
    na::normalize( &step ); // normalize step magnitude

    // Compute rate of change for quaternion
    let qDot:Quaternion<f64> = self.q * Quaternion::new(0.0f64, gyroscope[0], gyroscope[1], gyroscope[2])
        * 0.5 - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

    // Integrate to yield quaternion
    self.q = self.q + qDot * self.sample_period;
    self.quat = self.q;
    na::normalize( &self.quat );

    true
  }

  /// Updates the current state quaternion using 6dof IMU values.
  pub fn update_imu( &mut self, gyroscope: Vector3<f64>, accelerometer: Vector3<f64> ) -> bool {
    
    // Normalize accelerometer measurement
    if Norm::norm(&accelerometer) <= 0.0f64 {
      return false;
    }

    let accel = accelerometer.normalize();

    // Compute error between estimated and measured directly of gravity
    let v = Vector3::new(
              2.0f64*(self.q.i*self.q.k - self.q.w*self.q.j),
              2.0f64*(self.q.w*self.q.i + self.q.j*self.q.k),
                    self.q.w*self.q.w - self.q.i*self.q.i
                    - self.q.j*self.q.j + self.q.k*self.q.k
    );
    let error:Vector3<f64> = Cross::cross( &v, &accel );

    // Compute ramped Kp value used during init period
    //if self.kp_ramped > self.kp {
    //  self.int_error = Vector3::new(0.0f64, 0.0, 0.0);
    //  self.kp_ramped = self.kp_ramped - ( self.kp_init - self.kp ) / ( self.init_period / self.sample_period );
    //} else {
    //  self.kp_ramped = self.kp;
      self.int_error = self.int_error + error;
    //}

    // Apply feedback terms
    let Ref:Vector3<f64> = gyroscope - error*self.kp - self.int_error*self.ki;

    // Compute rate of change of quaternion
    let pDot:Quaternion<f64> = ( self.q * Quaternion::new( 0.0f64, Ref.x, Ref.y, Ref.z ) ) * 0.5f64;
    self.q = self.q + pDot * self.sample_period;
    self.q.normalize_mut();

    // Store conjugate
    self.quat = self.q;
    self.quat.conjugate_mut();

    true
  }

  pub fn reset( &mut self ) {
    self.kp_ramped = self.kp_init;
    self.int_error = Vector3::new(0.0f64, 0.0, 0.0);
    self.q = Quaternion::new(1.0f64, 0.0, 0.0, 0.0);
  }

  pub fn step_down( &mut self, kp_in: f64 ) {
    self.kp_ramped = self.kp;
    self.kp = kp_in;
  }

}

