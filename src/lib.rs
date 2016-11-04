#![allow(unused_attributes)]

#![crate_name = "ahrs"]

extern crate nalgebra as na;

pub use ahrs::{
    Ahrs
};
pub use madgwick::{
    Madgwick
};
pub use mahony::{
    Mahony
};

mod ahrs;
mod madgwick;
mod mahony;
