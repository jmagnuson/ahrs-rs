use ahrs::{Ahrs, Madgwick, Mahony};
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rand::{self, thread_rng, Rng};
use std::stringify;

macro_rules! get_rand_n(
  ($rng: ident, 1) => { $rng.gen(); };
  ($rng: ident, $n: expr) => { (0..$n).map(|_n| nalgebra::Vector3::new($rng.gen(), $rng.gen(), $rng.gen())).collect::<Vec<_>>() };
);

macro_rules! bench_ahrs(
    // boilerplate for each bench
    ($name: ident, $t: ident, $op: ident, $n: expr) => {
         fn $name(b: &mut Criterion) {
            let mut rng = thread_rng();
            _bench_function!(b, rng, $t, $op, $n);
         }
    };
);

macro_rules! _bench_function(
    // operation is `update`
    ($b: ident, $rng: ident, $t: ident, update, $n: expr) => {
        let (a, g, m) = ( get_rand_n!($rng, $n), get_rand_n!($rng, $n), get_rand_n!($rng, $n) );
        _bench_iterations!($b, $t, update, $n, a, g, m);
    };
    // operation is `update_imu`
    ($b: ident, $rng: ident, $t: ident, update_imu, $n: expr) => {
        let (a, g) = ( get_rand_n!($rng, $n), get_rand_n!($rng, $n) );
        _bench_iterations!($b, $t, update_imu, $n, a, g);
    };
);

macro_rules! _bench_iterations(
    // iterations is 1
    ($b: ident, $t: ident, $op: ident, 1, $( $x: expr ),* ) => {
        let mut ahrs = $t::default();
        $b.bench_function(stringify!($t $op 1), move |b| {
            b.iter(||  black_box(ahrs.$op( &$($x),* ).unwrap()));
        });
    };
    // iterations is $n, and $x is expanded based on input from `_bench_function` operation
    ($b: ident, $t: ident, $op: ident, $n: expr, $( $x: expr ),* )  => {
        $b.bench_function(stringify!($t $op $n), move |b| {
            b.iter(|| {
              let mut ahrs = $t::default();
              for n in 0..$n {
                  black_box(ahrs.$op( $( &$x[n] ),* ).unwrap());
              }
            })
        });
    };
);

bench_ahrs!(_bench_madgwick_update,           Madgwick, update,     1);
bench_ahrs!(_bench_madgwick_update_x1000,     Madgwick, update,     1000);
bench_ahrs!(_bench_madgwick_update_imu,       Madgwick, update_imu, 1);
bench_ahrs!(_bench_madgwick_update_imu_x1000, Madgwick, update_imu, 1000);
bench_ahrs!(_bench_mahony_update,             Mahony,   update,     1);
bench_ahrs!(_bench_mahony_update_x1000,       Mahony,   update,     1000);
bench_ahrs!(_bench_mahony_update_imu,         Mahony,   update_imu, 1);
bench_ahrs!(_bench_mahony_update_imu_x1000,   Mahony,   update_imu, 1000);

criterion_group!(
    benches,
    _bench_madgwick_update,
    _bench_madgwick_update_x1000,
    _bench_madgwick_update_imu,
    _bench_madgwick_update_imu_x1000,
    _bench_mahony_update,
    _bench_mahony_update_x1000,
    _bench_mahony_update_imu,
    _bench_mahony_update_imu_x1000,
);
criterion_main!(benches);
