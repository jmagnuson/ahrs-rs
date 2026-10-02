#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use crate::ahrs::{Ahrs, AhrsUpdate};
use crate::normalize::normalize;
use core::hash;
use nalgebra::{
    Matrix4, Matrix6, Quaternion, Scalar, UnitQuaternion, Vector2, Vector3, Vector4, Vector6,
};
use simba::{
    scalar::{ClosedNeg, RealField},
    simd::{SimdRealField, SimdValue},
};

/// Madgwick AHRS implementation.
///
/// # Example
/// ```
/// # use ahrs::Madgwick;
/// let mut ahrs = Madgwick::new(0.002390625f64, 0.1);
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
    /// Filter state quaternion.
    pub quat: UnitQuaternion<N>,
}

impl<N: Scalar + SimdValue + ClosedNeg + Eq + Copy> Eq for Madgwick<N> {}

impl<N: Scalar + SimdValue + ClosedNeg + Copy> PartialEq for Madgwick<N> {
    fn eq(&self, rhs: &Self) -> bool {
        self.sample_period == rhs.sample_period && self.beta == rhs.beta && self.quat == rhs.quat
    }
}

impl<N: Scalar + SimdValue + hash::Hash + Copy> hash::Hash for Madgwick<N> {
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
        *self
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
    /// // };
    /// ```
    fn default() -> Madgwick<f64> {
        Madgwick {
            sample_period: (1.0f64) / (256.0),
            beta: 0.1f64,
            quat: UnitQuaternion::new_unchecked(Quaternion::new(1.0f64, 0.0, 0.0, 0.0)),
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
    pub fn new(sample_period: N, beta: N) -> Self {
        Madgwick::new_with_quat(
            sample_period,
            beta,
            UnitQuaternion::new_unchecked(Quaternion::new(
                N::one(),
                N::zero(),
                N::zero(),
                N::zero(),
            )),
        )
    }

    /// Creates a new `Madgwick` AHRS instance with given quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `beta` - Filter gain.
    /// * `quat` - Existing filter state quaternion.
    pub fn new_with_quat(sample_period: N, beta: N, quat: UnitQuaternion<N>) -> Self {
        Madgwick {
            sample_period,
            beta,
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

    /// Filter state quaternion.
    pub fn quat(&self) -> UnitQuaternion<N> {
        self.quat
    }

    /// Mutable reference to filter state quaternion.
    pub fn quat_mut(&mut self) -> &mut UnitQuaternion<N> {
        &mut self.quat
    }
}

impl<N> Ahrs<N> for Madgwick<N>
where
    N: SimdRealField + Copy,
    N::Element: RealField,
{
    fn update(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
        magnetometer: &Vector3<N>,
    ) -> AhrsUpdate<'_, N> {
        let q = self.quat.as_ref();

        let zero: N = nalgebra::zero();
        let two: N = nalgebra::convert(2.0);
        let four: N = nalgebra::convert(4.0);
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let (accel, accel_valid) = normalize(accelerometer);

        // Normalize magnetometer measurement
        let (mag, mag_valid) = normalize(magnetometer);
        let valid = accel_valid & mag_valid;

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

        // A zero gradient contributes no correction in that lane.
        let (step, _) = normalize(&(J_t * F));

        // Compute rate of change for quaternion
        let qDot = q * Quaternion::from_parts(zero, *gyroscope) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        // Integrate to yield quaternion
        let quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);
        self.quat = quat.select(valid, self.quat);

        AhrsUpdate {
            quaternion: &self.quat,
            accelerometer_norm_zero: !accel_valid,
            magnetometer_norm_zero: accel_valid & !mag_valid,
        }
    }

    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> AhrsUpdate<'_, N> {
        let q = self.quat.as_ref();

        let zero: N = nalgebra::zero();
        let two: N = nalgebra::convert(2.0);
        let four: N = nalgebra::convert(4.0);
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let (accel, accel_valid) = normalize(accelerometer);

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

        // A zero gradient contributes no correction in that lane.
        let (step, _) = normalize(&(J_t * F));

        // Compute rate of change of quaternion
        let qDot = (q * Quaternion::from_parts(zero, *gyroscope)) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        // Integrate to yield quaternion
        let quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);
        self.quat = quat.select(accel_valid, self.quat);

        AhrsUpdate {
            quaternion: &self.quat,
            accelerometer_norm_zero: !accel_valid,
            magnetometer_norm_zero: zero.simd_gt(zero),
        }
    }

    fn update_gyro(&mut self, gyroscope: &Vector3<N>) -> &UnitQuaternion<N> {
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
