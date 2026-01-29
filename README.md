# lsm6dsv80x-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/lsm6dsv80x-rs
[crates-url]: https://crates.io/crates/lsm6dsv80x-rs
[bsd-badge]: https://img.shields.io/crates/l/lsm6dsv80x-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST LSM6DSV80X sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The LSM6DSV80X is a breakthrough in the world of wearable technology. Its ability to handle both high and low acceleration values, combined with its energy efficiency and advanced processing capabilities, makes it a sensor for anyone looking to
acquire data for in-depth analysis and achieve better performance in high-intensity impact and tracking activities in sports such as volleyball, soccer, tennis, boxing, or explosive jumps, and so forth. This IMU is a comprehensive solution for wearables, high-intensity impact and activity tracking, offering a blend of accuracy, integration, and efficiency.

The LSM6DSV80X is the world's first IMU to combine high-g (80 g) and low-g capabilities in a single package, integrating advanced features (edge processing and sensor fusion) and delivering consistent performance and
valuable data for tracking and high-intensity impact detection in sports wearables.

The device enables edge AI, leveraging on a finite state machine (FSM) for configurable motion tracking and a machine learning core (MLC) for context awareness with exportable AI features for wearable applications.

The LSM6DSV80X supports the adaptive self-configuration (ASC) feature, which allows automatically reconfiguring the device in real time based on the detection of a specific motion pattern or based on the output of
a specific decision tree configured in the MLC, without any intervention from the host processor.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/lsm6dsv80x.html](https://www.st.com/en/mems-and-sensors/lsm6dsv80x.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
lsm6dsv80x-rs = "2.0.0"
```

Or, add it directly from the terminal:

```sh
cargo add lsm6dsv80x-rs
```

## Usage

By default, the create exposes the **asynchronous** API, and it could be included using:
```rust
use lsm6dsv80x_rs::asynchronous as lsm6dsv80x;
use lsm6dsv80x::*;
use lsm6dsv80x::prelude::*;
```

### Blocking API (optional feature)

To use the **blocking** API instead of the asynchronous one, disable default features and enable the `blocking` feature in your Cargo.toml
```toml
[dependencies]
lsm6dsv80x-rs = { version = "2.0.0", default-features = false, features = ["blocking"] }
```
or from the terminal:
```sh
cargo add lsm6dsv80x-rs --no-default-features --features blocking
```

Then import the blocking API:
```rust
use lsm6dsv80x_rs::blocking as lsm6dsv80x;
use lsm6dsv80x::*;
use lsm6dsv80x::prelude::*;

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance, along with a timing peripheral.

An example with I2C:

```rust
let mut sensor = Lsm6dsv80x::new_i2c(i2c, I2CAddress::I2cAddL, delay);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.reset_set(Reset::RestoreCtrlRegs).unwrap();
let mut rst: Reset = Reset::RestoreCtrlRegs;
while rst != Reset::Ready {
    rst = sensor.reset_get().unwrap();
}
// Enable Block Data Update
sensor.block_data_update_set(1).unwrap();

// Set output data rate
sensor.xl_data_rate_set(DataRate::_1920hz).unwrap();
sensor.hg_xl_data_rate_set(HgXlDataRate::_960hz, 1).unwrap();
sensor.gy_data_rate_set(DataRate::_120hz).unwrap();

// Set full scale
sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();
sensor.hg_xl_full_scale_set(HgXlFullScale::_64g).unwrap();
sensor.gy_full_scale_set(GyFullScale::_2000dps).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**