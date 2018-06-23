# Embree-rs

[![Build Status](https://travis-ci.org/beltegeuse/embree-rs.svg?branch=master)](https://travis-ci.org/beltegeuse/embree-rs)

NOTE: Need Rust 1.25 at least to support ```repr(align(X))``` routine. To install this version, you can run the following command:

```RUSTUP_DIST_SERVER=https://dev-static.rust-lang.org rustup update stable```

## Description

A Rust wrapper for Intel's Embree raytracing library. It short terms goal is to provide easy intersection test in **Rust**. The goal of this library is **NOT** to support all Embree features for the moment. However, I will be pleased to merge any pull request!

This wrapper is used (and tested) in:

- rustlight: https://github.com/beltegeuse/rustlight

Please contact me if your project uses this wrapper so I can update this above list.

## TODO

- add documentation
- add an example of a simple Whitted ray tracing 
- add more intersections routines
- add phantom data
- expose different configurations (ray, scene, ...)

## Credits

The current code have been inspired from https://github.com/anderslanglands/embree-rs.
