# ahrs-rs

[![crates.io](https://img.shields.io/crates/v/ahrs.svg)](https://crates.io/crates/ahrs)
[![Build Status](https://github.com/jmagnuson/ahrs-rs/actions/workflows/test.yml/badge.svg)](https://github.com/jmagnuson/ahrs-rs/actions/workflows/test.yml)

A Rust port of Sebastian Madgwick's [AHRS algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/).

[Documentation](https://docs.rs/ahrs)

## Usage

Add ahrs-rs to your `Cargo.toml`:

```toml
[dependencies]
ahrs = "0.10"
```

Here's a simple example that updates the filter state with arbitrary sensor data:

```rust
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
        .into_result().unwrap();
    let (roll, pitch, yaw) = quat.euler_angles();

    // Do something with the updated state quaternion
    println!("pitch={}, roll={}, yaw={}", pitch, roll, yaw);
}
```

Crate [nalgebra](https://crates.io/crates/nalgebra) is also needed as a dependency for its algebraic types `Vector3` and `Quaternion`.

## Scalar and SIMD updates

`Ahrs` uses the same API for `f32`, `f64`, and SIMD values. `update` and
`update_imu` return `AhrsUpdate`: `is_ok()`/`is_err()` and the sensor error fields
are booleans for scalar inputs and masks for SIMD inputs. Scalar callers migrating
from 0.9 add `.into_result()` before `?`, `.unwrap()`, or other `Result` operations.
`update_gyro` still returns the quaternion directly.

Each SIMD lane is an independent stream, with its own orientation, sample period,
and gains. A failed lane preserves its quaternion and Mahony integral state;
valid lanes continue updating. Accelerometer errors take precedence within each
lane. A zero Madgwick gradient is successful gyro-only integration in that lane.
Inputs and intermediate norms must be finite and representable.

Enable `simd` and use Simba's `WideF32x4`, `WideF32x8`, or `WideF64x4` wrappers:

```toml
[dependencies]
ahrs = { version = "0.10", features = ["simd"] }
nalgebra = "0.35"
simba = "0.10.2"
```

```rust
use ahrs::{Ahrs, Madgwick};
use nalgebra::{SimdValue, Vector3};
use simba::simd::{SimdBool, WideF64x4 as F};

let mut filter = Madgwick::new(F::splat(1.0 / 256.0), F::splat(0.1));
let gyro = Vector3::new(F::from([0.1, 0.2, 0.3, 0.4]), F::splat(0.0), F::splat(0.0));
let accel = Vector3::new(
    F::from([0.1, 0.2, 0.0, 0.4]),
    F::splat(0.0),
    F::from([1.0, 1.0, 0.0, 1.0]),
);
let update = filter.update_imu(&gyro, &accel);
assert_eq!(update.is_ok().bitmask(), 0b1011); // lane 2 retains its state
let first_orientation = update.quaternion.as_ref().extract(0);
```

Pack x/y/z coordinates across streams as shown above. Consecutive samples of
one stream depend on each other and cannot occupy independent lanes. Raw `wide`
and `std::simd` values must be wrapped in Simba's types.

## Feature flags

- `std` (default): standard library support. Disable default features for `no_std`.
- `simd`: stable wide backend, also supported with `no_std`.
- `portable_simd`: enables `std` and Simba's nightly portable SIMD wrappers,
  such as `simba::simd::f64x4`. Tested with nightly-2026-10-01; this backend
  depends on unstable Rust APIs. Stable builds should select features explicitly
  rather than use `--all-features`.


### `field_access`

Gives [im]mutable access to inner algorithm struct fields that aren't normally exposed. The exposed
API is considered unstable for the time-being, but is nevertheless a useful feature. For example:

```rust
use ahrs::Madgwick;

let mut ahrs = Madgwick::default();

#[cfg(feature = "field_access")]
{
    let sample_period: &mut f64 = ahrs.sample_period_mut();
    *sample_period = 0.5;
}
```


## License

MIT

