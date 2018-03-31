#![cfg_attr(feature = "clippy", feature(plugin))]
#![cfg_attr(feature = "clippy", plugin(clippy))]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

// Include the binding
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

extern crate libc;
extern crate cgmath;

// Package
pub mod scene;
pub mod ray;
