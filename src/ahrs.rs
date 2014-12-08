
#![allow(non_snake_case)]
#![allow(dead_code)]       // unused placeholder variables

use na::{Vec2, Vec3, Vec6, Mat6, Norm, Cross, Quat};
use na;

pub struct AHRS {

  SamplePeriod: f64,
  pub Quaternion:   Quat<f64>,
  Kp:           f64,
  Ki:           f64,
  KpInit:       f64,
  InitPeriod:   f64,

  q:            Quat<f64>,
  IntError:     Vec3<f64>,
  KpRamped:     f64,

  // temporary until Update() is written properly
  Beta:         f64
}

impl AHRS {

  pub fn default() -> AHRS {
    AHRS { SamplePeriod: 0.00390625f64, Quaternion: Quat::new(1.0f64, 0.0, 0.0, 0.0),
           Kp: 1.0, Ki: 0.0, KpInit: 1.0, InitPeriod: 5.0, q: Quat::new(1.0f64, 0.0, 0.0, 0.0),
           IntError: Vec3::new(0.0f64, 0.0, 0.0), KpRamped: 0.0, Beta: 1.0
         }
  }

  pub fn Update( &mut self, Gyroscope: Vec3<f64>, Accelerometer: Vec3<f64>, Magnetometer: Vec3<f64> ) -> bool {
    
    // Nomralize accelerometer measurement
    if Norm::norm(&Accelerometer) > 0.0 {
      na::normalize( &Accelerometer ); 
    } else {
      return false;
    }
    
    // Nomralize magnetometer measurement
    if Norm::norm(&Magnetometer) > 0.0 {
      na::normalize( &Magnetometer ); 
    } else {
      return false;
    }

    // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
    let h = self.q * ( Quat::new(0.0f64, Magnetometer[0], Magnetometer[1], Magnetometer[2]) * self.Quaternion );
    let b = Quat::new( 0.0f64, Norm::norm(&Vec2::new(h[2], h[3])), 0.0, h[4] );

    // Gradient decent algorithm corrective step
    let F = Vec6::new( 
      2.0*(self.q[1]*self.q[3] - self.q[0]*self.q[2]) - Accelerometer[0],
      2.0*(self.q[0]*self.q[1] + self.q[2]*self.q[3]) - Accelerometer[1],
      2.0*(0.5 - self.q[1]*self.q[1] - self.q[2]*self.q[2]) - Accelerometer[2],
      2.0*b[1]*(0.5 - self.q[2]*self.q[2] - self.q[3]*self.q[3]) + 2.0*b[3]*(self.q[1]*self.q[3] - self.q[0]*self.q[2]) - Magnetometer[0],
      2.0*b[1]*(self.q[1]*self.q[2] - self.q[0]*self.q[3]) + 2.0*b[3]*(self.q[0]*self.q[1] + self.q[2]*self.q[3]) - Magnetometer[1],
      2.0*b[1]*(self.q[0]*self.q[2] + self.q[1]*self.q[3]) + 2.0*b[3]*(0.5 - self.q[1]*self.q[1] - self.q[2]*self.q[2]) - Magnetometer[2] );
   
    let J_t = Mat6::new(
      -2.0*self.q[2], 2.0*self.q[1], 0.0, -2.0*b[3]*self.q[2], -2.0*(b[1]*self.q[3]+b[3]*self.q[1]), 2.0*b[1]*self.q[2],
      2.0*self.q[3], 2.0*self.q[0], -4.0*self.q[1], 2.0*b[3]*self.q[3], 2.0*(b[1]*self.q[2]+b[3]*self.q[0]), 2.0*(b[1]*self.q[3]-2.0*b[3]*self.q[1]),
      -2.0*self.q[0], 2.0*self.q[3], -4.0*self.q[2], -2.0*(2.0*b[1]*self.q[2]-b[3]*self.q[0]), 2.0*(b[1]*self.q[1]+b[3]*self.q[3]), 2.0*(b[1]*self.q[0]-2.0*b[3]*self.q[2]),
      2.0*self.q[1], 2.0*self.q[2], 0.0, -2.0*(2.0*b[1]*self.q[3]+b[3]*self.q[1]), -2.0*(b[1]*self.q[0]+b[3]*self.q[2]), 2.0*b[1]*self.q[1],
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
   
    let step:Vec6<f64> = J_t * F;
    na::normalize( &step ); // normalize step magnitude

    // Compute rate of change for quaternion
    let qDot:Quat<f64> = self.q * Quat::new(0.0f64, Gyroscope[0], Gyroscope[1], Gyroscope[2]) 
        * 0.5 - Quat::new(step[0], step[1], step[2], step[3]) * self.Beta;

    // Integrate to yield quaternion
    self.q = self.q + qDot * self.SamplePeriod;
    self.Quaternion = self.q;
    na::normalize( &self.Quaternion );

    return true;
  }

  pub fn UpdateIMU( &mut self, Gyroscope: Vec3<f64>, Accelerometer: Vec3<f64> ) -> bool {

    // Normalize accelerometer measurement
    if Norm::norm(&Accelerometer) > 0.0 {
      na::normalize( &Accelerometer ); 
    } else {
      return false;
    }
    
    // Compute error between estimated and measured directly of gravity
    let v = Vec3::new( 
              2.0f64*self.q.i*self.q.k - self.q.w*self.q.j,
              2.0f64*self.q.w*self.q.i + self.q.j*self.q.k,
                     self.q.w*self.q.w - self.q.i*self.q.i 
                       - self.q.j*self.q.j + self.q.k*self.q.k );

    let error:Vec3<f64> = Cross::cross( &v, &Accelerometer );

    // Compute ramped Kp value used during init period
    //if self.KpRamped > self.Kp {
    //  self.IntError = Vec3::new(0.0f64, 0.0, 0.0);
    //  self.KpRamped = self.KpRamped - ( self.KpInit - self.Kp ) / ( self.InitPeriod / self.SamplePeriod );
    //} else {
    //  self.KpRamped = self.Kp;
      self.IntError = self.IntError + error;
    //}

    // Apply feedback terms
    let Ref:Vec3<f64> = Gyroscope - error*self.Kp - self.IntError*self.Ki;

    // Compute rate of change of quaternion
    let pDot:Quat<f64> = ( self.q * Quat::new( 0.0f64, Ref.x, Ref.y, Ref.z ) ) * 0.5f64;
    self.q = self.q + pDot * self.SamplePeriod;
    self.q.normalize();

    // Store conjugate
    self.Quaternion = self.q.clone();
    self.Quaternion.conjugate();

    return true;
  }

  pub fn Reset( &mut self ) {
    self.KpRamped = self.KpInit;
    self.IntError = Vec3::new(0.0f64, 0.0, 0.0);
    self.q = Quat::new(1.0f64, 0.0, 0.0, 0.0);
  }

  pub fn StepDown( &mut self, Kp_in: f64 ) {
    self.KpRamped = self.Kp;
    self.Kp = Kp_in;
  }

}

