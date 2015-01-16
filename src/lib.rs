#![crate_name = "ahrs"]
#![comment = "Madgwick AHRS algorithm"]
#![license = "GPLv2"]

//#![warn(missing_doc)]

#[cfg(test)]
extern crate test;

extern crate "nalgebra" as na;

pub use ahrs::{
    AHRS
};

pub mod ahrs;
