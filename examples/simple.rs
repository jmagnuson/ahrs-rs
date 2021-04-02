use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;
use std::f64;

fn main() {
    // Initialize filter with default values
    let mut ahrs = Madgwick::default();

    // Obtain sensor values from a source
    let gyroscope = Vector3::new(60.1, 30.2, 20.3);
    let accelerometer = Vector3::new(0.1, 0.2, 0.3);
    let magnetometer = Vector3::new(0.5, 0.6, 0.7);

    // Run inputs through AHRS filter (gyroscope must be radians/s)
    let quat = ahrs
        .update(
            &(gyroscope * (f64::consts::PI / 180.0)),
            &accelerometer,
            &magnetometer,
        )
        .unwrap();
    let (roll, pitch, yaw) = quat.euler_angles();

    // Do something with the updated state quaternion
    println!("pitch={}, roll={}, yaw={}", pitch, roll, yaw);
}
