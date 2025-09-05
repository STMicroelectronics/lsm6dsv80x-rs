use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    // Source file:
    // https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/lsm6dsv80x/lsm6dsv80x_glance.json
    let input_file = Path::new("lsm6dsv80x_glance.json");
    let output_file = Path::new("src/config.rs");
    parser::generate_rs_from_json(&input_file, &output_file, "GLANCE", "LSM6DSV80X", false);
    println!("cargo:rerun-if-changed=lsm6dsv80x_glance.json");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
