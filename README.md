# ahrs-rs

[![creates.io](http://meritbadge.herokuapp.com/ahrs)](https://crates.io/crates/ahrs)
[![Build Status](https://travis-ci.org/jmagnuson/ahrs-rs.svg?branch=master)](https://travis-ci.org/jmagnuson/ahrs-rs)

A Rust port of Sebastian Madgwick's [AHRS algorithm](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/).

## Usage

[Documentation](https://docs.rs/ahrs)

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
    ahrs.update(gyroscope * (f64::consts::PI/180.0), accelerometer, magnetometer);
    
    // Do something with the updated state quaternion
    println!("{}", ahrs.quat);
}

```