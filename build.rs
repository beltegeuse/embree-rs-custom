extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    // Tell cargo to tell rustc to link the system bzip2
    // shared library.
    println!("cargo:rustc-link-lib=embree");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let builder = bindgen::Builder::default()
        .clang_args(["-x", "c++", "-std=c++11"].iter())
        .enable_cxx_namespaces()
        // Remove max align as it does not pass the test
        .blacklist_type("max_align_t")
        .rust_target(bindgen::RustTarget::Stable_1_25)
        .header("wrapper.h");

    let commands = builder.command_line_flags();
    println!("Commands from bindgen: {:?}", commands);

    let bindings = builder.generate().expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = env::var("OUT_DIR").unwrap();
    let out_path = PathBuf::from(out_path);
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}