#![feature(test)]

extern crate test;
extern crate ahrs;
extern crate rand;

use test::Bencher;
use ahrs::{Ahrs, Madgwick, Mahony};
use rand::{Rng, Rand, ThreadRng};

// TODO: Bench with actual data and pull random sections.

pub fn get_random_n_vec<N: Rand>(rng: &mut ThreadRng, length: usize) -> Vec<N> {

    (0..length).map(|_n| rng.gen::<N>()).collect::<Vec<N>>()
}

#[bench]
fn bench_madgwick_update(b: &mut Bencher) {

    let mut ahrs = Madgwick::default();

    let mut rng = rand::thread_rng();

    let accel = rng.gen();
    let gyro = rng.gen();
    let mag = rng.gen();

    b.iter(|| ahrs.update(gyro, accel, mag));
}

#[bench]
fn bench_madgwick_update_imu(b: &mut Bencher) {

    let mut ahrs = Madgwick::default();

    let mut rng = rand::thread_rng();

    let accel = rng.gen();
    let gyro = rng.gen();

    b.iter(|| ahrs.update_imu(gyro, accel));
}

#[bench]
fn bench_mahony_update(b: &mut Bencher) {

    let mut ahrs = Mahony::default();

    let mut rng = rand::thread_rng();

    let accel = rng.gen();
    let gyro = rng.gen();
    let mag = rng.gen();

    b.iter(|| ahrs.update(gyro, accel, mag));
}

#[bench]
fn bench_mahony_update_imu(b: &mut Bencher) {

    let mut ahrs = Mahony::default();

    let mut rng = rand::thread_rng();

    let accel = rng.gen();
    let gyro = rng.gen();

    b.iter(|| ahrs.update_imu(gyro, accel));
}

#[bench]
fn bench_madgwick_update_x1000(b: &mut Bencher) {

    let iterations: usize = 1000;

    let mut rng = rand::thread_rng();

    let accels = get_random_n_vec(&mut rng, iterations);
    let gyros = get_random_n_vec(&mut rng, iterations);
    let mags = get_random_n_vec(&mut rng, iterations);

    b.iter(|| {
        let mut ahrs = Madgwick::default();
        for n in 0..iterations {
            ahrs.update(gyros[n], accels[n], mags[n]);
        }
    });
}

#[bench]
fn bench_madgwick_update_imu_x1000(b: &mut Bencher) {

    let iterations: usize = 1000;

    let mut rng = rand::thread_rng();

    let accels = get_random_n_vec(&mut rng, iterations);
    let gyros = get_random_n_vec(&mut rng, iterations);

    b.iter(|| {
        let mut ahrs = Madgwick::default();
        for n in 0..iterations {
            ahrs.update_imu(gyros[n], accels[n]);
        }
    });
}

#[bench]
fn bench_mahony_update_x1000(b: &mut Bencher) {

    let iterations: usize = 1000;

    let mut rng = rand::thread_rng();

    let accels = get_random_n_vec(&mut rng, iterations);
    let gyros = get_random_n_vec(&mut rng, iterations);
    let mags = get_random_n_vec(&mut rng, iterations);

    b.iter(|| {
        let mut ahrs = Mahony::default();
        for n in 0..iterations {
            ahrs.update(gyros[n], accels[n], mags[n]);
        }
    });
}

#[bench]
fn bench_mahony_update_imu_x1000(b: &mut Bencher) {

    let iterations: usize = 1000;

    let mut rng = rand::thread_rng();

    let accels = get_random_n_vec(&mut rng, iterations);
    let gyros = get_random_n_vec(&mut rng, iterations);

    b.iter(|| {
        let mut ahrs = Mahony::default();
        for n in 0..iterations {
            ahrs.update_imu(gyros[n], accels[n]);
        }
    });
}
