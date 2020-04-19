//! A collection of AHRS algorithms ported to Rust.

#![cfg_attr(not(feature = "std"), no_std)]
#![crate_name = "ahrs"]

extern crate nalgebra as na;


pub use crate::ahrs::{
    Ahrs
};
pub use crate::madgwick::{
    Madgwick
};
pub use crate::mahony::{
    Mahony
};

mod ahrs;
mod madgwick;
mod mahony;
