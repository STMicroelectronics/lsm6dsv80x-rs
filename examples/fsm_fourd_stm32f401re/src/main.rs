#![no_std]
#![no_main]

mod config;

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use lsm6dsv80x_rs::prelude::*;
use lsm6dsv80x_rs::*;
use panic_halt as _;
use stm32f4xx_hal::{
    gpio::{self, Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    pac::{self, interrupt},
    prelude::*,
    serial::{config::Config, Serial},
};
type IntPin = gpio::PC0<Input>;
use config::FOUR_D;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let tim1 = dp.TIM1.delay_us(&clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = Serial::tx(
        dp.USART2,
        tx_pin,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    )
    .unwrap();

    let mut int_pin = gpioc.pc0.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    delay.delay_ms(5);

    let mut sensor = Lsm6dsv80x::new_i2c(i2c, I2CAddress::I2cAddL, tim1);

    // Check device ID
    let id = sensor.device_id_get().unwrap();
    if id != lsm6dsv80x_rs::ID {
        writeln!(tx, "Unexpected device ID: {}", id).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor.reset_set(Reset::RestoreCtrlRegs).unwrap();
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = sensor.reset_get().unwrap();
    }

    for ucf_entry in FOUR_D {
        match ucf_entry.op {
            MemsUcfOp::Delay => {
                delay.delay_ms(ucf_entry.data.into());
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
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }
        let status = sensor.all_sources_get().unwrap();
        if status.fsm1 == 1 {
            let fsm_out = sensor.fsm_out_get().unwrap();
            match fsm_out.fsm_outs1 {
                0x10 => {
                    writeln!(tx, "Y down event").unwrap();
                }
                0x20 => {
                    writeln!(tx, "Y up event").unwrap();
                }
                0x40 => {
                    writeln!(tx, "X down event").unwrap();
                }
                0x80 => {
                    writeln!(tx, "X up event").unwrap();
                }
                _ => {}
            }
        }
    }
}

#[interrupt]
fn EXTI0() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain access to Peripheral and Clear Interrupt Pending Flag
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    });
}
