use ahrs::{Ahrs, AhrsError, AhrsUpdate, Madgwick, Mahony};
use nalgebra::{Quaternion, RealField, SimdRealField, SimdValue, UnitQuaternion, Vector3};
use num_traits::ToPrimitive;
use simba::simd::SimdBool;

fn pack<N: SimdRealField + Copy>(f: impl Fn(usize) -> f64) -> N
where
    N::Element: RealField,
{
    let mut value = N::zero();
    for lane in 0..N::LANES {
        value.replace(lane, nalgebra::convert(f(lane)));
    }
    value
}

fn check_quat<N>(actual: &UnitQuaternion<N>, expected: &UnitQuaternion<N::Element>, lane: usize)
where
    N: SimdRealField + Copy,
    N::Element: RealField + Copy + ToPrimitive,
{
    let tolerance = if core::mem::size_of::<N::Element>() == 4 {
        3e-6
    } else {
        1e-12
    };
    for coordinate in 0..4 {
        let a = actual.coords[coordinate].extract(lane).to_f64().unwrap();
        let e = expected.coords[coordinate].to_f64().unwrap();
        assert!(a.is_finite());
        assert!((a - e).abs() < tolerance, "lane {lane}: {a} != {e}");
    }
    let norm = actual.as_ref().norm().extract(lane).to_f64().unwrap();
    assert!((norm - 1.0).abs() < tolerance);
}

fn check_status<N: SimdRealField>(result: &AhrsUpdate<'_, N>, errors: &[Option<AhrsError>]) {
    let mut accel = 0;
    let mut mag = 0;
    for (lane, error) in errors.iter().enumerate() {
        match error {
            Some(AhrsError::AccelerometerNormZero) => accel |= 1 << lane,
            Some(AhrsError::MagnetometerNormZero) => mag |= 1 << lane,
            None => {}
        }
    }
    assert_eq!(result.accelerometer_norm_zero.bitmask(), accel);
    assert_eq!(result.magnetometer_norm_zero.bitmask(), mag);
    assert_eq!(result.is_err().bitmask(), accel | mag);
    assert_eq!(
        result.is_ok().bitmask(),
        ((1 << N::LANES) - 1) & !(accel | mag)
    );
}

/// Distinct parameters, rotations, sensor data, failures and recovery per lane.
fn parity<N>()
where
    N: SimdRealField + Copy,
    N::Element: RealField + Copy + ToPrimitive,
{
    let dt = pack::<N>(|i| 0.002 + i as f64 * 0.0005);
    let beta = pack::<N>(|i| 0.1 + i as f64 * 0.01);
    let kp = pack::<N>(|i| 0.5 + i as f64 * 0.1);
    let ki = pack::<N>(|i| 0.1 + i as f64 * 0.02);
    let start = UnitQuaternion::from_quaternion(Quaternion::new(
        N::one(),
        pack::<N>(|i| 0.1 * i as f64),
        N::zero(),
        N::zero(),
    ));
    for method in 0..3 {
        let mut md = Madgwick::new_with_quat(dt, beta, start);
        let mut mh = Mahony::new_with_quat(dt, kp, ki, start);
        let mut scalar_md: Vec<_> = (0..N::LANES)
            .map(|i| {
                Madgwick::new_with_quat(
                    dt.extract(i),
                    beta.extract(i),
                    UnitQuaternion::new_unchecked(start.as_ref().extract(i)),
                )
            })
            .collect();
        let mut scalar_mh: Vec<_> = (0..N::LANES)
            .map(|i| {
                Mahony::new_with_quat(
                    dt.extract(i),
                    kp.extract(i),
                    ki.extract(i),
                    UnitQuaternion::new_unchecked(start.as_ref().extract(i)),
                )
            })
            .collect();
        for tick in 0..24 {
            // Include an all-invalid batch, then recover with all lanes valid.
            let mode = |i| {
                if tick == 12 {
                    3
                } else if tick >= 13 {
                    0
                } else {
                    (tick + i) % 4
                }
            };
            let accel = Vector3::new(
                pack::<N>(|i| {
                    if mode(i) & 1 != 0 {
                        -0.0
                    } else {
                        0.1 + i as f64 * 0.05
                    }
                }),
                pack::<N>(|i| if mode(i) & 1 != 0 { 0.0 } else { 0.9 }),
                pack::<N>(|i| if mode(i) & 1 != 0 { 0.0 } else { 0.2 }),
            );
            let mag = Vector3::new(
                pack::<N>(|i| if mode(i) & 2 != 0 { 0.0 } else { 0.2 }),
                pack::<N>(|i| {
                    if mode(i) & 2 != 0 {
                        -0.0
                    } else {
                        -0.5 - i as f64 * 0.02
                    }
                }),
                pack::<N>(|i| if mode(i) & 2 != 0 { 0.0 } else { 0.1 }),
            );
            let gyro = Vector3::new(
                pack::<N>(|i| 0.4 + i as f64 * 0.01),
                N::zero(),
                pack::<N>(|_| 0.2),
            );
            let old_md = md;
            let old_mh = mh;
            let mut errors = Vec::new();
            for lane in 0..N::LANES {
                let a = accel.extract(lane);
                let g = gyro.extract(lane);
                let m = mag.extract(lane);
                let error = match method {
                    0 => {
                        let error = scalar_md[lane].update(&g, &a, &m).into_result().err();
                        assert_eq!(
                            scalar_mh[lane].update(&g, &a, &m).into_result().err(),
                            error
                        );
                        error
                    }
                    1 => {
                        let error = scalar_md[lane].update_imu(&g, &a).into_result().err();
                        assert_eq!(
                            scalar_mh[lane].update_imu(&g, &a).into_result().err(),
                            error
                        );
                        error
                    }
                    _ => {
                        scalar_md[lane].update_gyro(&g);
                        scalar_mh[lane].update_gyro(&g);
                        None
                    }
                };
                let expected_error = if method != 2 && mode(lane) & 1 != 0 {
                    Some(AhrsError::AccelerometerNormZero)
                } else if method == 0 && mode(lane) & 2 != 0 {
                    Some(AhrsError::MagnetometerNormZero)
                } else {
                    None
                };
                assert_eq!(error, expected_error);
                errors.push(error);
            }
            match method {
                0 => {
                    check_status(&md.update(&gyro, &accel, &mag), &errors);
                    check_status(&mh.update(&gyro, &accel, &mag), &errors);
                }
                1 => {
                    check_status(&md.update_imu(&gyro, &accel), &errors);
                    check_status(&mh.update_imu(&gyro, &accel), &errors);
                }
                _ => {
                    md.update_gyro(&gyro);
                    mh.update_gyro(&gyro);
                }
            }
            for lane in 0..N::LANES {
                check_quat(&md.quat, &scalar_md[lane].quat, lane);
                check_quat(&mh.quat, &scalar_mh[lane].quat, lane);
                if errors[lane].is_some() {
                    assert_eq!(
                        md.quat.as_ref().extract(lane),
                        old_md.quat.as_ref().extract(lane)
                    );
                    assert_eq!(
                        mh.quat.as_ref().extract(lane),
                        old_mh.quat.as_ref().extract(lane)
                    );
                }
                #[cfg(feature = "field_access")]
                {
                    let actual = mh.e_int().extract(lane);
                    let expected = scalar_mh[lane].e_int();
                    for j in 0..3 {
                        assert!(
                            (actual[j].to_f64().unwrap() - expected[j].to_f64().unwrap()).abs()
                                < 3e-6
                        );
                    }
                    if errors[lane].is_some() || method == 2 {
                        assert_eq!(actual, old_mh.e_int().extract(lane));
                    }
                }
            }
        }
    }
}

/// A zero gradient is gyro-only success; other lanes still receive correction.
fn mixed_gradients<N>()
where
    N: SimdRealField + Copy,
    N::Element: RealField + Copy + ToPrimitive,
{
    for use_mag in [false, true] {
        let dt = pack::<N>(|_| 1.0 / 256.0);
        let beta = pack::<N>(|_| 0.1);
        let mut filter = Madgwick::new(dt, beta);
        let mut gyro_only = filter;
        let accel = Vector3::new(
            pack::<N>(|i| if i % 2 == 0 { 0.0 } else { 0.2 }),
            N::zero(),
            N::one(),
        );
        let gyro = Vector3::new(pack::<N>(|_| 0.4), pack::<N>(|_| 0.2), pack::<N>(|_| 0.1));
        let mag = Vector3::new(N::one(), N::zero(), N::zero());
        let result = if use_mag {
            filter.update(&gyro, &accel, &mag)
        } else {
            filter.update_imu(&gyro, &accel)
        };
        assert!(result.is_ok().all());
        gyro_only.update_gyro(&gyro);
        for lane in 0..N::LANES {
            let mut scalar = Madgwick::new(dt.extract(lane), beta.extract(lane));
            let g = gyro.extract(lane);
            let a = accel.extract(lane);
            let m = mag.extract(lane);
            let result = if use_mag {
                scalar.update(&g, &a, &m)
            } else {
                scalar.update_imu(&g, &a)
            };
            assert!(result.into_result().is_ok());
            check_quat(&filter.quat, &scalar.quat, lane);
            if lane % 2 == 0 {
                assert_eq!(
                    filter.quat.as_ref().extract(lane),
                    gyro_only.quat.as_ref().extract(lane)
                );
            } else {
                assert_ne!(
                    filter.quat.as_ref().extract(lane),
                    gyro_only.quat.as_ref().extract(lane)
                );
            }
        }
    }
}

macro_rules! backend_tests {
    ($name:ident, $ty:ty) => {
        #[test]
        fn $name() {
            parity::<$ty>();
            mixed_gradients::<$ty>();
        }
    };
}

backend_tests!(scalar_f64, f64);
backend_tests!(scalar_f32, f32);

#[cfg(feature = "simd")]
mod wide {
    use super::*;
    backend_tests!(f64x4, simba::simd::WideF64x4);
    backend_tests!(f32x4, simba::simd::WideF32x4);
    backend_tests!(f32x8, simba::simd::WideF32x8);
}

#[cfg(feature = "portable_simd")]
mod portable {
    use super::*;
    backend_tests!(f64x1, simba::simd::f64x1);
    backend_tests!(f64x4, simba::simd::f64x4);
    backend_tests!(f32x4, simba::simd::f32x4);
}
