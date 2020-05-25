#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use crate::ahrs::{Ahrs, AhrsError};
use core::hash;
use nalgebra::{Quaternion, Scalar, Vector2, Vector3};
use simba::simd::{SimdRealField as RealField, SimdRealField, SimdValue};

/// Mahony AHRS implementation.
///
/// # Example
/// ```
/// # use ahrs::Mahony;
/// let mut ahrs = Mahony::new(0.002390625f64, 0.5, 0.0);
/// println!("mahony filter: {:?}", ahrs);
///
/// // Can now process IMU data using `Ahrs::update_imu`, etc.
/// ```
#[derive(Debug)]
pub struct Mahony<N: Scalar + SimdValue> {
    /// Expected sampling period, in seconds.
    sample_period: N,
    /// Proportional filter gain constant.
    kp: N,
    /// Integral filter gain constant.
    ki: N,
    /// Integral error vector.
    e_int: Vector3<N>,
    /// Filter state quaternion.
    pub quat: Quaternion<N>,
}

impl<N: SimdRealField + Eq> Eq for Mahony<N> where N::Element: SimdRealField {}

impl<N: SimdRealField> PartialEq for Mahony<N>
where
    N::Element: SimdRealField,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.sample_period == rhs.sample_period
            && self.kp == rhs.kp
            && self.ki == rhs.ki
            && self.e_int == rhs.e_int
            && self.quat == rhs.quat
    }
}

impl<N: SimdRealField + hash::Hash> hash::Hash for Mahony<N> {
    fn hash<H: hash::Hasher>(&self, state: &mut H) {
        self.sample_period.hash(state);
        self.kp.hash(state);
        self.ki.hash(state);
        self.e_int.hash(state);
        self.quat.hash(state);
    }
}

impl<N: Scalar + Copy + SimdValue> Copy for Mahony<N> {}

impl<N: Scalar + SimdValue> Clone for Mahony<N> {
    #[inline]
    fn clone(&self) -> Self {
        let sample_period = self.sample_period.clone();
        let kp = self.kp.clone();
        let ki = self.ki.clone();
        let e_int = self.e_int.clone();
        let quat = self.quat.clone();

        Mahony {
            sample_period,
            kp,
            ki,
            e_int,
            quat,
        }
    }
}

impl Default for Mahony<f64> {
    /// Creates a default `Mahony` AHRS instance with default filter parameters.
    ///
    /// ```
    /// # use ahrs::Mahony;
    /// # use nalgebra::{Quaternion, Vector4};
    /// dbg!(Mahony::default());
    ///
    /// // prints (roughly):
    /// //
    /// // Madgwick {
    /// //     sample_period: 1.0f64/256.0,
    /// //     kp: 0.5f64,
    /// //     ki: 0.0f64,
    /// //     e_int: Vector3 { x: 0.0f64, y: 0.0, z: 0.0 },
    /// //     quat: Quaternion { w: 1.0f64, i: 0.0, j: 0.0, k: 0.0 }
    /// // };
    /// ```
    fn default() -> Mahony<f64> {
        Mahony {
            sample_period: (1.0f64) / (256.0),
            kp: 0.5f64,
            ki: 0.0f64,
            e_int: Vector3::new(0.0, 0.0, 0.0),
            quat: Quaternion::new(1.0f64, 0.0, 0.0, 0.0),
        }
    }
}

impl<N: RealField> Mahony<N> {
    /// Creates a new Mahony AHRS instance with identity quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `kp` - Proportional filter gain constant.
    /// * `ki` - Integral filter gain constant.
    pub fn new(sample_period: N, kp: N, ki: N) -> Self {
        Mahony::new_with_quat(
            sample_period,
            kp,
            ki,
            Quaternion::from_parts(N::one(), nalgebra::zero::<nalgebra::Vector3<N>>()),
        )
    }

    /// Creates a new Mahony AHRS instance with given quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `kp` - Proportional filter gain constant.
    /// * `ki` - Integral filter gain constant.
    /// * `quat` - Existing filter state quaternion.
    pub fn new_with_quat(sample_period: N, kp: N, ki: N, quat: Quaternion<N>) -> Self {
        Mahony {
            sample_period,
            kp,
            ki,
            e_int: nalgebra::zero(),
            quat,
        }
    }
}

#[cfg(feature = "field_access")]
impl<N: Scalar + SimdValue + Copy> Mahony<N> {
    /// Expected sampling period, in seconds.
    pub fn sample_period(&self) -> N {
        self.sample_period
    }

    /// Mutable reference to expected sampling period, in seconds.
    pub fn sample_period_mut(&mut self) -> &mut N {
        &mut self.sample_period
    }

    /// Proportional filter gain constant.
    pub fn kp(&self) -> N {
        self.kp
    }

    /// Mutable reference to proportional filter gain constant.
    pub fn kp_mut(&mut self) -> &mut N {
        &mut self.kp
    }

    /// Integral filter gain constant.
    pub fn ki(&self) -> N {
        self.ki
    }

    /// Mutable reference to integral filter gain constant.
    pub fn ki_mut(&mut self) -> &mut N {
        &mut self.ki
    }

    /// Integral error vector.
    pub fn e_int(&self) -> Vector3<N> {
        self.e_int
    }

    /// Mutable reference to integral error vector.
    pub fn e_int_mut(&mut self) -> &mut Vector3<N> {
        &mut self.e_int
    }

    /// Filter state quaternion.
    pub fn quat(&self) -> Quaternion<N> {
        self.quat
    }

    /// Mutable reference to filter state quaternion.
    pub fn quat_mut(&mut self) -> &mut Quaternion<N> {
        &mut self.quat
    }
}

impl<N: simba::scalar::RealField> Ahrs<N> for Mahony<N> {
    fn update(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
        magnetometer: &Vector3<N>,
    ) -> Result<&Quaternion<N>, AhrsError> {
        let q = self.quat;

        let zero: N = nalgebra::zero();
        let two: N = nalgebra::convert(2.0);
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let accel = match accelerometer.try_normalize(zero) {
            Some(n) => n,
            None => {
                return Err(AhrsError::DivByZero);
            }
        };

        // Normalize magnetometer measurement
        let mag = match magnetometer.try_normalize(zero) {
            Some(n) => n,
            None => {
                return Err(AhrsError::DivByZero);
            }
        };

        // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
        let h = q * (Quaternion::from_parts(zero, mag) * q.conjugate());
        let b = Quaternion::new(zero, Vector2::new(h[0], h[1]).norm(), zero, h[2]);

        #[rustfmt::skip]
        let v = Vector3::new(
            two*( q[0]*q[2] - q[3]*q[1] ),
            two*( q[3]*q[0] + q[1]*q[2] ),
            q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]
        );

        #[rustfmt::skip]
        let w = Vector3::new(
            two*b[0]*(half - q[1]*q[1] - q[2]*q[2]) + two*b[2]*(q[0]*q[2] - q[3]*q[1]),
            two*b[0]*(q[0]*q[1] - q[3]*q[2])        + two*b[2]*(q[3]*q[0] + q[1]*q[2]),
            two*b[0]*(q[3]*q[1] + q[0]*q[2])        + two*b[2]*(half - q[0]*q[0] - q[1]*q[1])
        );

        let e: Vector3<N> = accel.cross(&v) + mag.cross(&w);

        // Error is sum of cross product between estimated direction and measured direction of fields
        if self.ki > zero {
            self.e_int += e * self.sample_period;
        } else {
            //Vector3::new(zero, zero, zero);
            self.e_int.x = zero;
            self.e_int.y = zero;
            self.e_int.z = zero;
        }

        // Apply feedback terms
        let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;

        // Compute rate of change of quaternion
        let qDot = q * Quaternion::from_parts(zero, gyro) * half;

        // Integrate to yield quaternion
        self.quat = (q + qDot * self.sample_period).normalize();

        Ok(&self.quat)
    }

    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> Result<&Quaternion<N>, AhrsError> {
        let q = self.quat;

        let zero: N = nalgebra::zero();
        let two: N = nalgebra::convert(2.0);
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let accel = match accelerometer.try_normalize(zero) {
            Some(n) => n,
            None => {
                return Err(AhrsError::DivByZero);
            }
        };

        #[rustfmt::skip]
        let v = Vector3::new(
            two*( q[0]*q[2] - q[3]*q[1] ),
            two*( q[3]*q[0] + q[1]*q[2] ),
            q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]
        );

        let e = accel.cross(&v);

        // Error is sum of cross product between estimated direction and measured direction of fields
        if self.ki > zero {
            self.e_int += e * self.sample_period;
        } else {
            self.e_int.x = zero;
            self.e_int.y = zero;
            self.e_int.z = zero;
        }

        // Apply feedback terms
        let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;

        // Compute rate of change of quaternion
        let qDot = q * Quaternion::from_parts(zero, gyro) * half;

        // Integrate to yield quaternion
        self.quat = (q + qDot * self.sample_period).normalize();

        Ok(&self.quat)
    }
}
