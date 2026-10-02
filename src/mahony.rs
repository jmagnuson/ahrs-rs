#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use crate::ahrs::{Ahrs, AhrsUpdate};
use crate::normalize::normalize;
use core::hash;
use nalgebra::{Quaternion, Scalar, UnitQuaternion, Vector2, Vector3};
use simba::{
    scalar::{ClosedNeg, RealField},
    simd::{SimdRealField, SimdValue},
};

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
pub struct Mahony<N: Scalar + SimdValue + Copy> {
    /// Expected sampling period, in seconds.
    sample_period: N,
    /// Proportional filter gain constant.
    kp: N,
    /// Integral filter gain constant.
    ki: N,
    /// Integral error vector.
    e_int: Vector3<N>,
    /// Filter state quaternion.
    pub quat: UnitQuaternion<N>,
}

impl<N: Scalar + SimdValue + ClosedNeg + Eq + Copy> Eq for Mahony<N> {}

impl<N: Scalar + SimdValue + ClosedNeg + Copy> PartialEq for Mahony<N> {
    fn eq(&self, rhs: &Self) -> bool {
        self.sample_period == rhs.sample_period
            && self.kp == rhs.kp
            && self.ki == rhs.ki
            && self.e_int == rhs.e_int
            && self.quat == rhs.quat
    }
}

impl<N: Scalar + SimdValue + hash::Hash + Copy> hash::Hash for Mahony<N> {
    fn hash<H: hash::Hasher>(&self, state: &mut H) {
        self.sample_period.hash(state);
        self.kp.hash(state);
        self.ki.hash(state);
        self.e_int.hash(state);
        self.quat.hash(state);
    }
}

impl<N: Scalar + Copy + SimdValue> Copy for Mahony<N> {}

impl<N: Scalar + SimdValue + Copy> Clone for Mahony<N> {
    #[inline]
    fn clone(&self) -> Self {
        *self
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
            quat: UnitQuaternion::new_unchecked(Quaternion::new(1.0f64, 0.0, 0.0, 0.0)),
        }
    }
}

impl<N: SimdRealField + Copy> Mahony<N> {
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
            UnitQuaternion::new_unchecked(Quaternion::new(
                N::one(),
                N::zero(),
                N::zero(),
                N::zero(),
            )),
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
    pub fn new_with_quat(sample_period: N, kp: N, ki: N, quat: UnitQuaternion<N>) -> Self {
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
    pub fn quat(&self) -> UnitQuaternion<N> {
        self.quat
    }

    /// Mutable reference to filter state quaternion.
    pub fn quat_mut(&mut self) -> &mut UnitQuaternion<N> {
        &mut self.quat
    }
}

impl<N> Ahrs<N> for Mahony<N>
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
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let (accel, accel_valid) = normalize(accelerometer);

        // Normalize magnetometer measurement
        let (mag, mag_valid) = normalize(magnetometer);
        let valid = accel_valid & mag_valid;

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

        // Error is sum of cross product between estimated direction and measured direction of fields
        let e: Vector3<N> = accel.cross(&v) + mag.cross(&w);

        // Integrate error
        let e_int = self.e_int + e * self.sample_period;

        // Apply feedback terms
        let gyro = *gyroscope + e * self.kp + e_int * self.ki;

        // Compute rate of change of quaternion
        let qDot = q * Quaternion::from_parts(zero, gyro) * half;

        // Integrate to yield quaternion
        let quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);
        self.quat = quat.select(valid, self.quat);
        self.e_int = e_int.select(valid, self.e_int);

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
        let half: N = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let (accel, accel_valid) = normalize(accelerometer);

        #[rustfmt::skip]
        let v = Vector3::new(
            two*( q[0]*q[2] - q[3]*q[1] ),
            two*( q[3]*q[0] + q[1]*q[2] ),
            q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2]
        );

        // Error is estimated direction direction of fields
        let e = accel.cross(&v);

        // Integrate error
        let e_int = self.e_int + e * self.sample_period;

        // Apply feedback terms
        let gyro = *gyroscope + e * self.kp + e_int * self.ki;

        // Compute rate of change of quaternion
        let qDot = q * Quaternion::from_parts(zero, gyro) * half;

        // Integrate to yield quaternion
        let quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);
        self.quat = quat.select(accel_valid, self.quat);
        self.e_int = e_int.select(accel_valid, self.e_int);

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
