//! Main functions and other top-level code needed to operate

// Copyright (c) 2023 Charles M. Thompson
//
// This file is part of Ms. Baker's firmware.
//
// Ms. Baker's firmware is free software: you can redistribute it and/or modify it under
// the terms only of version 3 of the GNU General Public License as published
// by the Free Software Foundation
//
// Ms. Baker's firmware is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along with
// Ms. Baker's firmware(in a file named COPYING).
// If not, see <https://www.gnu.org/licenses/>.

#![no_std]
#![no_main]

mod sdio;
mod errors;

use crate::sdio::Sdio4bit;

use rp2040_hal as hal;

use core::panic::PanicInfo;
use hal::pac;

use hal::dma::DMAExt;

use hal::pio::PIOExt;
use embedded_hal::blocking::i2c::WriteRead;
use fugit::RateExtU32;

use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::clocks::Clock;
use cortex_m::delay::Delay;

use arrayvec::ArrayString;
use core::fmt::Write;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const IMU_ADDR: u8 = 0b1101010;
const IMU_CHECK_REG: u8 = 0x0F;
const IMU_CHECK_VAL: u8 = 0b01101100;

/// Core 0 main function, the entrypoint for our code
/// Everything starts here!
#[rp2040_hal::entry]
fn main_0() -> ! {
    // ===================================================================== //
    // STEP 1, INITIALIZATION!                                               //
    // ===================================================================== //
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let dma = pac.DMA.split(&mut pac.RESETS);

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Initialize GPIO
    let mut pin_led = pins.gpio25.into_push_pull_output();
    // Initialize the I2C1 line
    // !TODO! says it's I2C0 on the schematic, figure out why
    let pin_i2c1_sda = pins.gpio6.into_mode::<hal::gpio::FunctionI2C>();
    let pin_i2c1_scl = pins.gpio7.into_mode::<hal::gpio::FunctionI2C>();
    let mut i2c1 = hal::I2C::i2c1(
        pac.I2C1,
        pin_i2c1_sda,
        pin_i2c1_scl,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Initialize the SD card pins
    let pin_sd_clk = pins.gpio0.into_mode::<hal::gpio::FunctionPio0>(); 
    let pin_sd_cmd = pins.gpio5.into_mode::<hal::gpio::FunctionPio0>();
    let pin_sd_dat0 = pins.gpio1.into_mode::<hal::gpio::FunctionPio0>();
    let pin_sd_dat1 = pins.gpio2.into_mode::<hal::gpio::FunctionPio0>();
    let pin_sd_dat2 = pins.gpio3.into_mode::<hal::gpio::FunctionPio0>();
    let pin_sd_dat3 = pins.gpio4.into_mode::<hal::gpio::FunctionPio0>();

    // Get their ids
    let pin_sd_clk_id = pin_sd_clk.id().num;
    let pin_sd_cmd_id = pin_sd_cmd.id().num;
    let pin_sd_dat0_id = pin_sd_dat0.id().num;

    // Initialize the sd card
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut sd_controller = Sdio4bit::new(
        pio,
        dma.ch0,
        delay,
        sm0,
        sm1,
        pin_sd_clk_id,
        pin_sd_cmd_id, 
        pin_sd_dat0_id,
    );

    sd_controller.init().unwrap();

    // ===================================================================== //
    // STEP 1.1, SELF CHECKS!                                                //
    // ===================================================================== //

    // Check the IMU
    let mut response: [u8; 1] = [0; 1];
    // Remember you have to write to i2c to read from i2c...
    i2c1.write_read(IMU_ADDR, &[IMU_CHECK_REG], &mut response)
        .unwrap();

    if response[0] != IMU_CHECK_VAL {
        panic!("IMU NOT OK");
    }

    // !TODO! Self check for sd card
    


    // !TODO! Initialize core 1

    // ===================================================================== //
    // STEP 2, WAIT!                                                         //
    // ===================================================================== //
    //sd_controller.start_block_tx().unwrap();
    //sd_controller.wait_tx().unwrap();
    // Turn the LED solid to signify all is good!
    
    assert_eq!(sd_controller.working_block.unwrap()[0], 0xFFFF_FFFF);

    pin_led.set_high().unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}

/// Core 1 main function, called by core 0
fn main_1() {
    todo!()
}

static xmorse: [u8; 256] = [
    67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 12, 77, 78, 79, 80, 81, 82, 83, 84,
    85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 0, 35, 65, 98, 99, 100,
    66, 24, 58, 59, 101, 102, 17, 36, 15, 103, 104, 64, 105, 106, 107, 108,
    109, 110, 111, 112, 62, 33, 113, 114, 115, 32, 116, 30, 37, 48, 50, 42, 44,
    40, 27, 28, 63, 45, 41, 39, 46, 34, 43, 53, 49, 38, 29, 60, 61, 31, 117,
    51, 118, 52, 119, 54, 120, 121, 122, 4, 23, 20, 11, 1, 19, 21, 7, 8, 56,
    26, 10, 14, 6, 3, 22, 55, 9, 5, 2, 13, 25, 18, 47, 16, 57, 123, 124, 125,
    126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
    141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155,
    156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
    171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,
    186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
    201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,
    216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
    231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245,
    246, 247, 248, 249, 250, 251, 252, 253, 254, 255
];

#[panic_handler]
fn panic(panic_info: &PanicInfo) -> ! {
    unsafe {

        let mut message_string = ArrayString::<256>::new();

        write!(&mut message_string, "{}", panic_info);

        let message = &message_string;

        let mut pac = pac::Peripherals::steal();
        let core = pac::CorePeripherals::steal();

        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = hal::Sio::new(pac.SIO);

        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // Initialize GPIO
        let mut pin_led = pins.gpio25.into_push_pull_output();


        let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        loop {

            // Flash it at 4hz
            for _ in 0 .. 4*10 {
                pin_led.set_high();
                delay.delay_ms(125);
                pin_led.set_low();
                delay.delay_ms(125);
            }

            delay.delay_ms(1000);
            for byte in message.as_bytes() {
                let mut byte: u8 = xmorse[*byte as usize];
                // Flash quick twice to indicate new byte
                pin_led.set_high();
                delay.delay_ms(125);
                pin_led.set_low();
                delay.delay_ms(125);
                pin_led.set_high();
                delay.delay_ms(125);
                pin_led.set_low();
                delay.delay_ms(1000);

                while byte > 0 {
                    let delaytime = match (byte & 1) > 0 {
                        true => 125,
                        false => 500,
                    };
                    pin_led.set_high();
                    delay.delay_ms(delaytime);
                    pin_led.set_low();
                    delay.delay_ms(1000-delaytime);
                    byte >>= 1;
                }
            }
        }
    }
}
