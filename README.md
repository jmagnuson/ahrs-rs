# ahrs-rs

[![crates.io](http://meritbadge.herokuapp.com/ahrs)](https://crates.io/crates/ahrs)
[![Build Status](https://travis-ci.org/jmagnuson/ahrs-rs.svg?branch=master)](https://travis-ci.org/jmagnuson/ahrs-rs)

A Rust port of Sebastian Madgwick's [AHRS algorithm](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/).

[Documentation](https://docs.rs/ahrs)

## Usage

Add ahrs-rs to your `Cargo.toml`:

```toml
[dependencies]
ahrs = "0.2.2"
```

Here's a simple example that updates the filter state with arbitrary sensor data:

```rust
extern crate nalgebra as na;
extern crate ahrs;

use ahrs::{Ahrs, Madgwick};
use na::{Vector3, Quaternion};
use std::f64;

fn main() {

    // Initialize filter with default values
    let mut ahrs = Madgwick::default();

    // Obtain sensor values from a source
    let gyroscope = Vector3::new(60.1, 30.2, 20.3);
    let accelerometer = Vector3::new(0.1, 0.2, 0.3);
    let magnetometer = Vector3::new(0.5, 0.6, 0.7);
    
    // Run inputs through AHRS filter (gyroscope must be radians/s)
    let quat: &Quaternion<f64> = ahrs.update( &(gyroscope * (f64::consts::PI/180.0)),
                                              &accelerometer,
                                              &magnetometer ).unwrap();
    
    // Do something with the updated state quaternion
    println!("{}", quat);
}

```

Crate [nalgebra](https://crates.io/crates/nalgebra) is also needed as a dependency for its algebraic types `Vector3` and `Quaternion`.

## Feature flags

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

GPLv3

