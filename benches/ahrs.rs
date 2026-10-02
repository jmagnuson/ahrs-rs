use ahrs::{Ahrs, Madgwick, Mahony};
use criterion::{Criterion, criterion_group, criterion_main};
use rand::{self, RngExt, rng};
use std::hint::black_box;
use std::stringify;

macro_rules! get_rand_n(
  ($rng: ident, 1) => { $rng.random(); };
  ($rng: ident, $n: expr) => { (0..$n).map(|_n| nalgebra::Vector3::new($rng.random(), $rng.random(), $rng.random())).collect::<Vec<_>>() };
);

macro_rules! bench_ahrs(
    // boilerplate for each bench
    ($name: ident, $t: ident, $op: ident, $n: expr) => {
         fn $name(b: &mut Criterion) {
            let mut rng = rng();
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
            b.iter(||  black_box(ahrs.$op( &$($x),* ).into_result().unwrap()));
        });
    };
    // iterations is $n, and $x is expanded based on input from `_bench_function` operation
    ($b: ident, $t: ident, $op: ident, $n: expr, $( $x: expr ),* )  => {
        $b.bench_function(stringify!($t $op $n), move |b| {
            b.iter(|| {
              let mut ahrs = $t::default();
              for n in 0..$n {
                  black_box(ahrs.$op( $( &$x[n] ),* ).into_result().unwrap());
              }
            })
        });
    };
);

bench_ahrs!(_bench_madgwick_update, Madgwick, update, 1);
bench_ahrs!(_bench_madgwick_update_x1000, Madgwick, update, 1000);
bench_ahrs!(_bench_madgwick_update_imu, Madgwick, update_imu, 1);
bench_ahrs!(_bench_madgwick_update_imu_x1000, Madgwick, update_imu, 1000);
bench_ahrs!(_bench_mahony_update, Mahony, update, 1);
bench_ahrs!(_bench_mahony_update_x1000, Mahony, update, 1000);
bench_ahrs!(_bench_mahony_update_imu, Mahony, update_imu, 1);
bench_ahrs!(_bench_mahony_update_imu_x1000, Mahony, update_imu, 1000);

criterion_group!(
    benches,
    bench_simd,
    _bench_madgwick_update,
    _bench_madgwick_update_x1000,
    _bench_madgwick_update_imu,
    _bench_madgwick_update_imu_x1000,
    _bench_mahony_update,
    _bench_mahony_update_x1000,
    _bench_mahony_update_imu,
    _bench_mahony_update_imu_x1000,
);

#[cfg(feature = "simd")]
fn bench_simd(c: &mut Criterion) {
    use criterion::Throughput;
    use nalgebra::{SimdValue, Vector3};
    use simba::simd::WideF64x4 as F;

    // Compare four independent streams, including the cost of packing when
    // inputs arrive as scalar vectors. Every benchmark processes four samples.
    let gyro = core::array::from_fn::<_, 4, _>(|i| Vector3::new(0.1 + i as f64 * 0.02, 0.2, 0.3));
    let accel = core::array::from_fn::<_, 4, _>(|i| Vector3::new(0.1 + i as f64 * 0.02, 0.9, 0.2));
    let mag = core::array::from_fn::<_, 4, _>(|i| Vector3::new(0.2, -0.5 - i as f64 * 0.02, 0.1));
    let pack = |v: &[Vector3<f64>; 4]| {
        Vector3::new(
            F::from(core::array::from_fn(|i| v[i][0])),
            F::from(core::array::from_fn(|i| v[i][1])),
            F::from(core::array::from_fn(|i| v[i][2])),
        )
    };
    let (g, a, m) = (pack(&gyro), pack(&accel), pack(&mag));
    macro_rules! compare {
        ($filter:ident, $method:ident, $scalar:expr, $simd:expr, $($input:ident => $packed:ident),+) => {{
            let mut group = c.benchmark_group(concat!(stringify!($filter), "_", stringify!($method), "_four_streams"));
            group.throughput(Throughput::Elements(4));
            let mut scalar = [$scalar; 4];
            group.bench_function("scalar", |b| b.iter(|| {
                for (i, filter) in scalar.iter_mut().enumerate() {
                    black_box(filter.$method($(black_box(&$input[i])),+).into_result().unwrap());
                }
            }));
            let mut simd = $simd;
            group.bench_function("wide_prepacked", |b| b.iter(|| {
                black_box(simd.$method($(black_box(&$packed)),+).is_ok());
            }));
            let mut simd = $simd;
            group.bench_function("wide_with_packing", |b| b.iter(|| {
                $(let $packed = pack(black_box(&$input));)+
                black_box(simd.$method($(&$packed),+).is_ok());
            }));
            group.finish();
        }};
    }
    compare!(Madgwick, update, Madgwick::new(1.0 / 256.0, 0.1),
        Madgwick::new(F::splat(1.0 / 256.0), F::splat(0.1)), gyro => g, accel => a, mag => m);
    compare!(Madgwick, update_imu, Madgwick::new(1.0 / 256.0, 0.1),
        Madgwick::new(F::splat(1.0 / 256.0), F::splat(0.1)), gyro => g, accel => a);
    compare!(Mahony, update, Mahony::new(1.0 / 256.0, 0.5, 0.1),
        Mahony::new(F::splat(1.0 / 256.0), F::splat(0.5), F::splat(0.1)), gyro => g, accel => a, mag => m);
    compare!(Mahony, update_imu, Mahony::new(1.0 / 256.0, 0.5, 0.1),
        Mahony::new(F::splat(1.0 / 256.0), F::splat(0.5), F::splat(0.1)), gyro => g, accel => a);
}

#[cfg(not(feature = "simd"))]
fn bench_simd(_: &mut Criterion) {}

criterion_main!(benches);
