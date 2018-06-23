extern crate bindgen;

use std::env;
use std::io::{Read, Write};
use std::path::PathBuf;

fn main() {
    // Tell cargo to tell rustc to link the system bzip2
    // shared library.
    println!("cargo:rustc-link-lib=embree");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        .clang_args(["-x", "c++", "-std=c++11"].iter())
        .enable_cxx_namespaces()
        .whitelist_function("rtc.*")
        .whitelist_type("RTC.*")
        .whitelist_var("rtc.*")
        .whitelist_var("RTC.*")
        .rust_target(bindgen::RustTarget::Stable_1_25)
        .header("wrapper.h")
        .bitfield_enum("RTC.*Flags")
        .rustified_enum("RTCDeviceProperty")
        .rustified_enum("RTCError")
        .rustified_enum("RTCBufferType")
        .rustified_enum("RTCGeometryType")
        .rustified_enum("RTCSubdivisionMode")
        .rustified_enum("RTCFormat")
        .rustified_enum("RTCBuildQuality")
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = env::var("OUT_DIR").unwrap();
    let out_path = PathBuf::from(out_path);
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    println!(
        "Generating output bindings: {:?}",
        out_path.join("bindings.rs")
    );

    // Simplify bindgen output
    let names = [
        "RTC_FORMAT_",
        "RTC_BUILD_QUALITY_",
        "RTC_DEVICE_PROPERTY_",
        "RTC_ERROR_",
        "RTC_BUFFER_TYPE_",
        "RTC_GEOMETRY_TYPE_",
        "RTC_SUBDIVISION_MODE_",
        "RTC_INTERSECT_CONTEXT_FLAG_",
        "RTC_CURVE_FLAG_",
        "RTC_SCENE_FLAG_",
        "RTC_BUILD_FLAG_",
    ];

    // Open and read the file entirely
    let mut src = std::fs::File::open(out_path.join("bindings.rs"))
        .expect("Impossible to reopen binding file");
    let mut data = String::new();
    src.read_to_string(&mut data)
        .expect("Impossible to read the binding file");
    drop(src); // Close the file early

    // Do the replacement
    for name in &names {
        data = data.replace(name, "");
    }

    // Recreate the file and dump the processed contents to it
    let mut dst = std::fs::File::create(out_path.join("bindings.rs"))
        .expect("Impossible to rewrite the binding file");
    dst.write(data.as_bytes())
        .expect("Impossible to write the binding file");
}
