#![cfg_attr(feature = "clippy", feature(plugin))]
#![cfg_attr(feature = "clippy", plugin(clippy))]

extern crate libc;
extern crate cgmath;
#[macro_use] extern crate bitflags;
#[macro_use] extern crate enum_primitive;

// Package
pub mod scene;
pub mod ray;
mod ffi;
