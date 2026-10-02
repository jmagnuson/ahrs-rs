use ahrs::{Ahrs, Madgwick};
use nalgebra::{SimdValue, Vector3};
use simba::simd::{SimdBool, WideF64x4 as F};

fn main() {
    // Each lane holds one independent stream, with gyroscope values in rad/s.
    let mut filter = Madgwick::new(F::splat(1.0 / 256.0), F::splat(0.1));
    let gyro = Vector3::new(F::from([0.1, 0.2, 0.3, 0.4]), F::splat(0.0), F::splat(0.0));
    let accel = Vector3::new(
        F::from([0.1, 0.2, 0.0, 0.4]),
        F::splat(0.0),
        F::from([1.0, 1.0, 0.0, 1.0]),
    );
    let update = filter.update_imu(&gyro, &accel);
    assert_eq!(update.is_ok().bitmask(), 0b1011);
    println!(
        "accelerometer failures: {:04b}",
        update.accelerometer_norm_zero.bitmask()
    );
    for lane in 0..4 {
        println!(
            "stream {lane}: {:?}",
            update.quaternion.as_ref().extract(lane)
        );
    }
}
