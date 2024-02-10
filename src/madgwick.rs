#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use crate::ahrs::{Ahrs, AhrsError};
use core::hash;
use nalgebra::{
    Matrix4, Matrix6, Quaternion, Scalar, UnitQuaternion, Vector2, Vector3, Vector4, Vector6,
};
use simba::simd::{SimdRealField, SimdValue};

/// Madgwick AHRS implementation.
///
/// # Example
/// ```
/// # use ahrs::Madgwick;
/// let mut ahrs = Madgwick::new(0.002390625f64, 0.1, 1e-3);
/// println!("madgwick filter: {:?}", ahrs);
///
/// // Can now process IMU data using `Ahrs::update_imu`, etc.
/// ```
#[derive(Debug)]
pub struct Madgwick<N: Scalar + SimdValue + Copy> {
    /// Expected sampling period, in seconds.
    sample_period: N,
    /// Filter gain.
    beta: N,
    /// Normalization stabilizer
    delta: N,
    /// Filter state quaternion.
    pub quat: UnitQuaternion<N>,
}

impl<N: SimdRealField + Eq + Copy> Eq for Madgwick<N> where N::Element: SimdRealField + Copy {}

impl<N: SimdRealField + Copy> PartialEq for Madgwick<N>
where
    N::Element: SimdRealField + Copy,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.sample_period == rhs.sample_period && self.beta == rhs.beta && self.quat == rhs.quat
    }
}

impl<N: SimdRealField + hash::Hash + Copy> hash::Hash for Madgwick<N> {
    fn hash<H: hash::Hasher>(&self, state: &mut H) {
        self.sample_period.hash(state);
        self.beta.hash(state);
        self.quat.hash(state);
    }
}

impl<N: Scalar + Copy + SimdValue> Copy for Madgwick<N> {}

impl<N: Scalar + SimdValue + Copy> Clone for Madgwick<N> {
    #[inline]
    fn clone(&self) -> Self {
        let sample_period = self.sample_period;
        let beta = self.beta;
        let delta = self.delta;
        let quat = self.quat;

        Madgwick {
            sample_period,
            beta,
            delta,
            quat,
        }
    }
}

impl Default for Madgwick<f64> {
    /// Creates a new `Madgwick` instance with default filter parameters.
    ///
    /// ```
    /// # use ahrs::Madgwick;
    /// # use nalgebra::{Quaternion, Vector4};
    /// dbg!(Madgwick::default());
    ///
    /// // prints (roughly):
    /// //
    /// // Madgwick {
    /// //     sample_period: 1.0f64/256.0,
    /// //     beta: 0.1f64,
    /// //     quat: Quaternion { w: 1.0f64, i: 0.0, j: 0.0, k: 0.0 },
    /// //     delta: 1e-9
    /// // };
    /// ```
    fn default() -> Madgwick<f64> {
        Madgwick {
            sample_period: (1.0f64) / (256.0),
            beta: 0.1f64,
            quat: UnitQuaternion::new_unchecked(Quaternion::new(1.0f64, 0.0, 0.0, 0.0)),
            delta: nalgebra::convert(1e-9),
        }
    }
}

impl<N: Scalar + SimdValue + num_traits::One + num_traits::Zero + Copy> Madgwick<N> {
    /// Creates a new `Madgwick` AHRS instance with identity quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `beta` - Filter gain.
    /// * `delta` - Normalization stabilizer.
    pub fn new(sample_period: N, beta: N, delta : N) -> Self {
        Madgwick::new_with_quat(
            sample_period,
            beta,
            delta,
            UnitQuaternion::new_unchecked(Quaternion::new(
                N::one(),
                N::zero(),
                N::zero(),
                N::zero(),
            ))
        )
    }

    /// Creates a new `Madgwick` AHRS instance with given quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `beta` - Filter gain.
    /// * `delta` - Normalization stabilizer.
    /// * `quat` - Existing filter state quaternion.
    pub fn new_with_quat(sample_period: N, beta: N, delta : N, quat: UnitQuaternion<N>) -> Self {
        Madgwick {
            sample_period,
            beta,
            delta,
            quat,
        }
    }
}

#[cfg(feature = "field_access")]
impl<N: Scalar + SimdValue + Copy> Madgwick<N> {
    /// Expected sampling period, in seconds.
    pub fn sample_period(&self) -> N {
        self.sample_period
    }

    /// Mutable reference to expected sampling period, in seconds.
    pub fn sample_period_mut(&mut self) -> &mut N {
        &mut self.sample_period
    }

    /// Filter gain.
    pub fn beta(&self) -> N {
        self.beta
    }

    /// Mutable reference to filter gain.
    pub fn beta_mut(&mut self) -> &mut N {
        &mut self.beta
    }

    /// Normalization stabilizer.
    pub fn delta(&self) -> N {
        self.delta
    }

    /// Mutable reference to normalization stabilizer.
    pub fn delta_mut(&mut self) -> &mut N {
        &mut self.delta
    }

    /// Filter state quaternion.
    pub fn quat(&self) -> UnitQuaternion<N> {
        self.quat
    }

    /// Mutable reference to filter state quaternion.
    pub fn quat_mut(&mut self) -> &mut UnitQuaternion<N> {
        &mut self.quat
    }
}

impl<N: simba::scalar::RealField + Copy> Ahrs<N> for Madgwick<N> {
    fn update(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
        magnetometer: &Vector3<N>,
    ) -> Result<&UnitQuaternion<N>, AhrsError> {
        let q = self.quat.as_ref();

        let zero: N = nalgebra::zero();
        let two: N = nalgebra::convert(2.0);
        let four: N = nalgebra::convert(4.0);
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let accel = match accelerometer.try_normalize(zero) {
            Some(n) => n,
            None => return Err(AhrsError::AccelerometerNormZero),
        };

        // Normalize magnetometer measurement
        let mag = match magnetometer.try_normalize(zero) {
            Some(n) => n,
            None => return Err(AhrsError::MagnetometerNormZero),
        };

        // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
        let h = q * (Quaternion::from_parts(zero, mag) * q.conjugate());
        let b = Quaternion::new(zero, Vector2::new(h[0], h[1]).norm(), zero, h[2]);

        // Gradient descent algorithm corrective step
        #[rustfmt::skip]
        let F = Vector6::new(
            two*(       q[0]*q[2] - q[3]*q[1]) - accel[0],
            two*(       q[3]*q[0] + q[1]*q[2]) - accel[1],
            two*(half - q[0]*q[0] - q[1]*q[1]) - accel[2],
            two*b[0]*(half - q[1]*q[1] - q[2]*q[2]) + two*b[2]*(q[0]*q[2] - q[3]*q[1]) - mag[0],
            two*b[0]*(q[0]*q[1] - q[3]*q[2])        + two*b[2]*(       q[3]*q[0] + q[1]*q[2]) - mag[1],
            two*b[0]*(q[3]*q[1] + q[0]*q[2])        + two*b[2]*(half - q[0]*q[0] - q[1]*q[1]) - mag[2]
        );

        #[rustfmt::skip]
        let J_t = Matrix6::new(
            -two*q[1], two*q[0],       zero,                -two*b[2]*q[1], -two*b[0]*q[2]+two*b[2]*q[0], two*b[0]*q[1],
             two*q[2], two*q[3], -four*q[0],                 two*b[2]*q[2],  two*b[0]*q[1]+two*b[2]*q[3], two*b[0]*q[2]-four*b[2]*q[0],
            -two*q[3], two*q[2], -four*q[1], -four*b[0]*q[1]-two*b[2]*q[3],  two*b[0]*q[0]+two*b[2]*q[2], two*b[0]*q[3]-four*b[2]*q[1],
             two*q[0], two*q[1],       zero, -four*b[0]*q[2]+two*b[2]*q[0], -two*b[0]*q[3]+two*b[2]*q[1], two*b[0]*q[0],
             zero, zero, zero, zero, zero, zero,
             zero, zero, zero, zero, zero, zero
        );

        // Normalize step with stabilizing parameter
        let prod = J_t * F;
        let step = prod.unscale(prod.norm() + self.delta);

        // Compute rate of change for quaternion
        let qDot = q * Quaternion::from_parts(zero, *gyroscope) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        // Integrate to yield quaternion
        self.quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);

        Ok(&self.quat)
    }

    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> Result<&UnitQuaternion<N>, AhrsError> {
        let q = self.quat.as_ref();

        let zero: N = nalgebra::zero();
        let two: N = nalgebra::convert(2.0);
        let four: N = nalgebra::convert(4.0);
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let accel = match accelerometer.try_normalize(zero) {
            Some(n) => n,
            None => return Err(AhrsError::AccelerometerNormZero)
        };

        // Gradient descent algorithm corrective step
        #[rustfmt::skip]
        let F = Vector4::new(
            two*(       q[0]*q[2] - q[3]*q[1]) - accel[0],
            two*(       q[3]*q[0] + q[1]*q[2]) - accel[1],
            two*(half - q[0]*q[0] - q[1]*q[1]) - accel[2],
            zero
        );

        #[rustfmt::skip]
        let J_t = Matrix4::new(
            -two*q[1], two*q[0],       zero, zero,
             two*q[2], two*q[3], -four*q[0], zero,
            -two*q[3], two*q[2], -four*q[1], zero,
             two*q[0], two*q[1],       zero, zero
        );

        // Normalize step with stabilizing parameter
        let prod = J_t * F;
        let step = prod.unscale(prod.norm() + self.delta);

        // Compute rate of change of quaternion
        let qDot = (q * Quaternion::from_parts(zero, *gyroscope)) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        // Integrate to yield quaternion
        self.quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);

        Ok(&self.quat)
    }

    fn update_gyro(
        &mut self,
        gyroscope: &Vector3<N>
    ) -> &UnitQuaternion<N> {
        let q = self.quat.as_ref();

        let zero: N = nalgebra::zero();
        let half: N = nalgebra::convert(0.5);

        // Compute rate of change for quaternion
        let qDot = q * Quaternion::from_parts(zero, *gyroscope) * half;

        // Integrate to yield quaternion
        self.quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);

        &self.quat
    }
}
