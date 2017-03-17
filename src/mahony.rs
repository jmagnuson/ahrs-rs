#![allow(non_snake_case)]

use na::{Vector2, Vector3, Quaternion, try_normalize, norm};
use na;
use ahrs::Ahrs;
use alga::general::Real;


/// Mahony AHRS implementation.
#[derive(Eq, PartialEq, Clone, Debug, Hash, Copy)]
pub struct Mahony<N: Real> {

    /// Expected sampling period, in seconds.
    sample_period: N,
    /// Proportional filter gain constant.
    kp: N,
    /// Integral filter gain constant.
    ki: N,
    /// Integral error vector.
    e_int: Vector3<N>,
    /// Filter state quaternion.
    pub quat: Quaternion<N>

}

impl Default for Mahony<f64> {

    /// Creates a default `Mahony` AHRS instance with default filter parameters:
    ///
    /// ```rust,ignore
    /// Mahony {
    ///     sample_period: 1.0f64/256.0,
    ///     kp: 0.5f64,
    ///     ki: 0.0f64,
    ///     e_int: Vector3 { x: 0.0f64, y: 0.0, z: 0.0 },
    ///     quat: Quaternion { w: 1.0f64, i: 0.0, j: 0.0, k: 0.0 }
    /// }
    /// ```
    fn default() -> Mahony<f64> {
        Mahony { sample_period: (1.0f64)/(256.0),
                 kp: 0.5f64,
                 ki: 0.0f64,
                 e_int: Vector3::new(0.0, 0.0, 0.0),
                 quat: Quaternion::new(1.0f64, 0.0, 0.0, 0.0),
        }
    }
}


impl<N: Real> Mahony<N> {

  /// Creates a new Mahony AHRS instance with identity quaternion.
  ///
  /// # Arguments
  ///
  /// * `sample_period` - The expected sensor sampling period in seconds.
  /// * `kp` - Proportional filter gain constant.
  /// * `ki` - Integral filter gain constant.
  ///
  /// # Example
  ///
  /// ```
  /// extern crate ahrs;
  ///
  /// use ahrs::Mahony;
  ///
  /// fn main() {
  ///
  ///     let ahrs = Mahony::new(0.002390625f64, 0.5, 0.0);
  /// }
  /// ```
  pub fn new(sample_period: N, kp: N, ki: N) -> Self {
    Mahony::new_with_quat(sample_period, kp, ki, Quaternion::from_parts(N::one(), na::zero::<na::Vector3<N>>()))
  }

  /// Creates a new Mahony AHRS instance with given quaternion.
  ///
  /// # Arguments
  ///
  /// * `sample_period` - The expected sensor sampling period in seconds.
  /// * `kp` - Proportional filter gain constant.
  /// * `ki` - Integral filter gain constant.
  /// * `quat` - Existing filter state quaternion.
  ///
  /// # Example
  ///
  /// ```
  /// extern crate nalgebra as na;
  /// extern crate ahrs;
  ///
  /// use na::Quaternion;
  /// use ahrs::Mahony;
  ///
  /// fn main() {
  ///
  ///     let ahrs = Mahony::new_with_quat( 0.002390625f64,
  ///                                       0.5,
  ///                                       0.0,
  ///                                       Quaternion::new(1.0, 0.0, 0.0, 0.0));
  /// }
  /// ```
  pub fn new_with_quat(sample_period: N, kp: N, ki: N, quat: Quaternion<N>) -> Self {
    Mahony { sample_period: sample_period,
             kp: kp,
             ki: ki,
             e_int: na::zero(),
             quat: quat
    }
  }

}
impl<N: Real> Ahrs<N> for Mahony<N> {

  fn update( &mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N>, magnetometer: &Vector3<N> ) -> Result<&Quaternion<N>, &str> {

    let q = self.quat;
    
    let zero: N = na::zero();
    let two: N = na::convert(2.0);
    let half: N = na::convert(0.5);

    // Normalize accelerometer measurement
    let accel = match try_normalize(accelerometer, zero) {
        Some(n) => n,
        None => { return Err("Accelerometer norm divided by zero."); }
    };
    
    // Normalize magnetometer measurement
    let mag = match try_normalize(magnetometer, zero) {
        Some(n) => n,
        None => { return Err("Magnetometer norm divided by zero."); }
    };

    // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
    let h = q * ( Quaternion::from_parts(zero, mag) * q.conjugate() );
    let b = Quaternion::new( zero, norm(&Vector2::new(h[1], h[2])), zero, h[3] );
 
    let v = Vector3::new(
      two*( q[1]*q[3] - q[0]*q[2] ),
      two*( q[0]*q[1] + q[2]*q[3] ),
      q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3] );

    let w = Vector3::new(
        two*b[1]*(half - q[2]*q[2] - q[3]*q[3]) + two*b[3]*(q[1]*q[3] - q[0]*q[2]),
        two*b[1]*(q[1]*q[2] - q[0]*q[3]) + two*b[3]*(q[0]*q[1] + q[2]*q[3]),
        two*b[1]*(q[0]*q[2] + q[1]*q[3]) + two*b[3]*(half - q[1]*q[1] - q[2]*q[2]));

    let e: Vector3<N> = accel.cross(&v) + mag.cross(&w);
    
    // Error is sum of cross product between estimated direction and measured direction of fields
    if self.ki > zero  {
        self.e_int += e * self.sample_period;
    } else {
        //Vector3::new(zero, zero, zero);
        self.e_int.x = zero; self.e_int.y = zero; self.e_int.z = zero;
    }
    
    // Apply feedback terms
    let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;

    // Compute rate of change of quaternion
    let qDot = q * Quaternion::from_parts(zero, gyro) * half;

    // Integrate to yield quaternion
    self.quat = na::normalize( &(q + qDot * self.sample_period) );

    Ok(&self.quat)
  }

  fn update_imu( &mut self, gyroscope: &Vector3<N>, accelerometer: &Vector3<N> ) -> Result<&Quaternion<N>, &str> {

    let q = self.quat;
    
    let zero: N = na::zero();
    let two: N = na::convert(2.0);
    let half: N = na::convert(0.5);

    // Normalize accelerometer measurement
    let accel = match try_normalize(accelerometer, zero) {
        Some(n) => n,
        None => { return Err("Accelerometer norm divided by zero."); }
    };

    let v = Vector3::new(
      two*( q[1]*q[3] - q[0]*q[2] ),
      two*( q[0]*q[1] + q[2]*q[3] ),
      q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3] );

    let e = accel.cross(&v);

    // Error is sum of cross product between estimated direction and measured direction of fields
    if self.ki > zero {
        self.e_int += e * self.sample_period;
//        self.e_int += e;
    } else {
        self.e_int.x = zero; self.e_int.y = zero; self.e_int.z = zero;
    }

    // Apply feedback terms
    let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;

    // Compute rate of change of quaternion
    let qDot = q * Quaternion::from_parts(zero, gyro) * half;

    // Integrate to yield quaternion
    self.quat = na::normalize( &(q + qDot * self.sample_period) );

    Ok(&self.quat)
  }

}

