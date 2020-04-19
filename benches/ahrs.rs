#![feature(test)]

extern crate test;

use rand;

use test::Bencher;
use ahrs::{Ahrs, Madgwick, Mahony};
use rand::{Rng};

macro_rules! get_rand_n(
  ($rng: ident, 1) => { $rng.gen(); };
  ($rng: ident, $n: expr) => { (0..$n).map(|_n| $rng.gen::<_>()).collect::<Vec<_>>(); };
);

macro_rules! bench_ahrs(
    // boilerplate for each bench
    ($name: ident, $t: ident, $op: ident, $n: expr) => {
        #[bench]
         fn $name(b: &mut Bencher) {
            let mut rng = rand::thread_rng();
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
        $b.iter(|| test::black_box( ahrs.$op( &$($x),* ) ).unwrap());
    };
    // iterations is $n, and $x is expanded based on input from `_bench_function` operation
    ($b: ident, $t: ident, $op: ident, $n: expr, $( $x: expr ),* )  => {
        $b.iter(|| {
          let mut ahrs = $t::default();
          for n in 0..$n {
              test::black_box(ahrs.$op( $( &$x[n] ),* ).unwrap());
          }
        })
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

