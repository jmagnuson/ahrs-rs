#![feature(test)]

extern crate ahrs;
extern crate nalgebra as na;

extern crate test;

use test::Bencher;
use na::Vector3;
use ahrs::Ahrs;

extern crate rand;
use rand::Rng;

// TODO: Bench with actual data and pull random sections.

#[bench]
fn bench_update(b: &mut Bencher) {

    let mut ahrs = Ahrs::default();
    let accel = Vector3::new(-3.102632460623745e-02f64, 3.049616351072091e-02, 9.721632321637426e-01);
    let gyro = Vector3::new(0.0f64, 0.0, 0.0);
    let mag = Vector3::new(1.4f64, 1.4, 1.2);

    b.iter(|| ahrs.update(gyro, accel, mag));
}

#[bench]
fn bench_update_imu(b: &mut Bencher) {

    let mut ahrs = Ahrs::default();
    let accel = Vector3::new(-3.102632460623745e-02f64, 3.049616351072091e-02, 9.721632321637426e-01);
    let gyro = Vector3::new(0.0f64, 0.0, 0.0);

    b.iter(|| ahrs.update_imu(gyro, accel));
}

#[bench]
fn bench_update_x1000(b: &mut Bencher) {

    let iterations: usize = 1000;

    let mut ahrs0 = Ahrs::default();

    let mut rng = rand::thread_rng();


    let accels0: Vec<Vector3<f64>> = (0..iterations*10)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();
    let gyros0: Vec<Vector3<f64>> = (0..iterations*10)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();
    let mags0: Vec<Vector3<f64>> = (0..iterations*10)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();

    // seed values into ahrs
    for n in 0..iterations*10 {
        ahrs0.update(gyros0[n], accels0[n], mags0[n]);
    }

    let accels: Vec<Vector3<f64>> = (0..iterations)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();
    let gyros: Vec<Vector3<f64>> = (0..iterations)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();
    let mags: Vec<Vector3<f64>> = (0..iterations)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();

    b.iter(|| {
        let mut ahrs = ahrs0.clone();
        for n in 0..iterations {
            ahrs.update(gyros[n], accels[n], mags[n]);
        }
    });
}

#[bench]
fn bench_update_imu_x1000(b: &mut Bencher) {

    let iterations: usize = 1000;

    let mut ahrs0 = Ahrs::default();

    let mut rng = rand::thread_rng();


    let accels0: Vec<Vector3<f64>> = (0..iterations*10)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();
    let gyros0: Vec<Vector3<f64>> = (0..iterations*10)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();

    // seed values into ahrs
    for n in 0..iterations*10 {
        ahrs0.update_imu(gyros0[n], accels0[n]);
    }

    let accels: Vec<Vector3<f64>> = (0..iterations)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();
    let gyros: Vec<Vector3<f64>> = (0..iterations)
        .map(|_n| Vector3::new(rng.gen::<f64>(), rng.gen::<f64>(), rng.gen::<f64>()))
        .collect::<Vec<Vector3<f64>>>();

    b.iter(|| {
        let mut ahrs = ahrs0.clone();
        for n in 0..iterations {
            ahrs.update_imu(gyros[n], accels[n]);
        }
    });
}
