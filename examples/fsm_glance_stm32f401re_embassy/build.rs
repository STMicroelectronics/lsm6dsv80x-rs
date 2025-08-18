use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    let input_file = Path::new("lsm6dsv80x_glance_detection.ucf");
    let output_file = Path::new("src/config.rs");
    parser::generate_rs_from_ucf(&input_file, &output_file, "GLANCE");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
