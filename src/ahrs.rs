use nalgebra::{Scalar, UnitQuaternion, Vector3};
use simba::simd::SimdValue;

/// A sensor measurement could not be normalized.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AhrsError {
    AccelerometerNormZero,
    MagnetometerNormZero,
}

/// The filter state after an update, with an error mask for each sensor.
///
/// For scalar `f32`/`f64`, masks are ordinary `bool` values. SIMD masks describe
/// independent filter lanes: successful lanes advance, while failed lanes keep
/// their previous quaternion and integral state. If both sensors have zero
/// norm in a lane, only the accelerometer error is reported.
#[derive(Debug)]
#[must_use = "inspect the error masks or convert a scalar update with into_result()"]
pub struct AhrsUpdate<'a, N: Scalar + SimdValue> {
    /// Updated quaternion; failed lanes contain their previous state.
    pub quaternion: &'a UnitQuaternion<N>,
    /// Lanes whose accelerometer measurement had zero norm.
    pub accelerometer_norm_zero: N::SimdBool,
    /// Lanes with valid acceleration whose magnetometer had zero norm.
    /// Always false for `update_imu`.
    pub magnetometer_norm_zero: N::SimdBool,
}

impl<N: Scalar + SimdValue> AhrsUpdate<'_, N> {
    /// Mask of successfully updated lanes (a `bool` for scalar values).
    pub fn is_ok(&self) -> N::SimdBool {
        !self.is_err()
    }

    /// Mask of lanes whose state was preserved because of a sensor error.
    pub fn is_err(&self) -> N::SimdBool {
        self.accelerometer_norm_zero | self.magnetometer_norm_zero
    }
}

impl<'a, N: Scalar + SimdValue<SimdBool = bool>> AhrsUpdate<'a, N> {
    /// Converts a scalar update to a conventional `Result`.
    ///
    /// ```
    /// use ahrs::{Ahrs, Madgwick};
    /// use nalgebra::Vector3;
    /// let mut filter = Madgwick::default();
    /// let gyro = Vector3::zeros();
    /// let accel = Vector3::new(0.0, 0.0, 1.0);
    /// let quaternion = filter.update_imu(&gyro, &accel).into_result().unwrap();
    /// assert_eq!(*quaternion, nalgebra::UnitQuaternion::identity());
    /// ```
    pub fn into_result(self) -> Result<&'a UnitQuaternion<N>, AhrsError> {
        if self.accelerometer_norm_zero {
            Err(AhrsError::AccelerometerNormZero)
        } else if self.magnetometer_norm_zero {
            Err(AhrsError::MagnetometerNormZero)
        } else {
            Ok(self.quaternion)
        }
    }
}

/// An AHRS filter over scalar or SIMD numeric values.
///
/// Each SIMD lane represents an independent stream, including its parameters
/// and orientation. Gyroscope measurements must be in radians per second.
/// Sensor normalization uses a zero threshold. Inputs and intermediate norms
/// should be finite and representable; non-finite inputs are not supported.
pub trait Ahrs<N: Scalar + SimdValue> {
    /// Updates orientation from gyroscope, accelerometer and magnetometer data.
    ///
    /// Lanes with a zero sensor norm retain all previous state. Accelerometer
    /// errors take precedence over magnetometer errors within each lane.
    fn update(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
        magnetometer: &Vector3<N>,
    ) -> AhrsUpdate<'_, N>;

    /// Updates orientation from gyroscope and accelerometer data.
    ///
    /// Lanes with a zero accelerometer norm retain all previous state.
    fn update_imu(
        &mut self,
        gyroscope: &Vector3<N>,
        accelerometer: &Vector3<N>,
    ) -> AhrsUpdate<'_, N>;

    /// Updates every lane from gyroscope data alone and returns the orientation.
    /// Does not change Mahony's integral error state.
    fn update_gyro(&mut self, gyroscope: &Vector3<N>) -> &UnitQuaternion<N>;
}
