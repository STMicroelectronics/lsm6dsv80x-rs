#![no_std]
#![no_main]

mod config;

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use config::GLANCE;
use lsm6dsv80x_rs::prelude::*;
use lsm6dsv80x_rs::*;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    // Configure the interrupt pin (if needed) and obtain handler.
    // On the Nucleo FR401 the interrupt pin is connected to pin PC0.
    let interrupt = Input::new(p.PC0, Pull::None);
    let mut interrupt = ExtiInput::new(interrupt, p.EXTI0);

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

    let mut sensor = Lsm6dsv80x::new_i2c(i2c, I2CAddress::I2cAddL, delay.clone());

    // Check device ID
    let whoami = sensor.device_id_get().unwrap();
    if whoami != ID {
        writeln!(&mut msg, "Device ID mismatch: {:#02x}", whoami).unwrap();
        let _ = tx.blocking_write(msg.as_bytes());
        msg.clear();
        loop {}
    }

    // Restore default configuration
    sensor.reset_set(Reset::RestoreCtrlRegs).unwrap();
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = sensor.reset_get().unwrap();
    }

    for ucf_entry in GLANCE {
        match ucf_entry.op {
            MemsUcfOp::Delay => {
                delay.delay_ms(ucf_entry.data as u8);
            }
            MemsUcfOp::Write => {
                sensor
                    .write_to_register(ucf_entry.address as u8, &[ucf_entry.data])
                    .unwrap();
            }
            _ => {}
        }
    }

    loop {
        interrupt.wait_for_rising_edge().await;
        let status = sensor.all_sources_get().unwrap();
        if status.fsm1 == 1 {
            let fsm_out = sensor.fsm_out_get().unwrap();
            match fsm_out.fsm_outs1 {
                0x20 => {
                    writeln!(&mut msg, "deglance event").unwrap();
                }
                0x8 => {
                    writeln!(&mut msg, "Glance event").unwrap();
                }
                _ => {}
            }
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
