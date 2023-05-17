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

mod errors;

use ape_table_trig::{abs_f32, trig_table_gen_f32, TrigTableF32};
use bytemuck::{bytes_of, bytes_of_mut};
use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use hal::gpio::bank0::{Gpio6, Gpio7};
use hal::gpio::{Function, FunctionI2C, Pin};
use hal::multicore::{Multicore, Stack};
use hal::sio::{Spinlock29, Spinlock30};
use hal::timer::{Alarm, Instant};
use libm::atan2f;
use pac::I2C1;
use rp_sdio::sdio::Sdio4bit;

use ape_mbr::{Partition, PartitionId, MBR};
use rp2040_hal as hal;

use core::panic::PanicInfo;
use core::str::FromStr;
use hal::{pac, Timer, I2C};

use hal::dma::{Channel, DMAExt, CH0};

use embedded_hal::blocking::i2c::WriteRead;
use fugit::{
    MicrosDuration, MicrosDurationU32, MicrosDurationU64, RateExtU32, SecsDurationU64,
    TimerInstantU64,
};
use hal::pio::PIOExt;

use cortex_m::delay::Delay;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::clocks::Clock;

use arrayvec::ArrayString;
use core::fmt::Write as FmtWrite;

use ape_fatfs::fs::{FileSystem, FsOptions};
use embedded_io::blocking::Write;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const IMU_ADDR: u8 = 0b1101010;
const IMU_CHECK_REG: u8 = 0x0F;
const IMU_CHECK_VAL: u8 = 0b01101100;

static mut CORE1_STACK: Stack<4096> = Stack::new();

// SIO Command Constants

const SIO_CMD_READY: u32 = 0x00;
const SIO_CMD_PANIC: u32 = 0x01;
const SIO_CMD_TELEM: u32 = 0x02;

// IMU config
const IMU_CFG_1: u8 = 0b1000_0000;
const IMU_CFG_2: u8 = 0b1000_0000;
const IMU_CFG_3: u8 = 0b0100_0100;

// Config starts at 10h
const IMU_CFG: [u8; 4] = [0x10, IMU_CFG_1, IMU_CFG_2, IMU_CFG_3];
// Telemetry starts at 20h
const IMU_TELM_REG: u8 = 0x20;
// Acceleration registers start at 28h
const IMU_ACCEL_REG: u8 = 0x28;

// Conversion multipliers
const IMU_ACCEL_MULT_4G: f32 = 1.196e-3; // 0.122e-3g/LSB -> 1.196e-3m/s^2/LSB
const IMU_RATE_MULT_250DPS: f32 = -1.527e-4; // 8.750e-3dps/LSB -> 1.527e-4 rad/s/LSB. Negative
                                             // since the rate is read counterclockwise while we
                                             // need our heading to be clockwise
const IMU_TEMP_MULT: f32 = 3.906e-3; // 3.906e-3c/LSB
const GRAVITY_MS2: f32 = 9.80665;
const ZERO_THRESH: f32 = 0.1; // Threshold to zero a heading.Prevents gymbal lock at startup.

// It only needs to be accurate enough down to 7.636e-5 radians.
// Make the size equal to 2π/7.636e-5 aka 82284 (divided by 4, so 41142)
const TRIG_TBL: [f32; 20571] = trig_table_gen_f32!(20571);

/// Core 0 main function, the entrypoint for our code
///
/// Core 0 is in charge of starting core 1 and output. Ouput in the case of
/// base Ms. Baker being SD card I/O and turning the status LED green
///
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

    let mut sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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

    // Initialize GPIO
    let mut pin_led = pins.gpio25.into_push_pull_output();

    // Initialize the I2C1 line
    // !TODO! says it's I2C0 on the schematic, figure out why
    let pin_i2c1_sda = pins.gpio6.into_mode::<hal::gpio::FunctionI2C>();
    let pin_i2c1_scl = pins.gpio7.into_mode::<hal::gpio::FunctionI2C>();
    let i2c1 = hal::I2C::i2c1(
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
    // Unused but still required
    let _pin_sd_dat1 = pins.gpio2.into_mode::<hal::gpio::FunctionPio0>();
    let _pin_sd_dat2 = pins.gpio3.into_mode::<hal::gpio::FunctionPio0>();
    let _pin_sd_dat3 = pins.gpio4.into_mode::<hal::gpio::FunctionPio0>();

    // Get their ids
    let pin_sd_clk_id = pin_sd_clk.id().num;
    let pin_sd_cmd_id = pin_sd_cmd.id().num;
    let pin_sd_dat0_id = pin_sd_dat0.id().num;

    // Initialize the sd card
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut sd_controller = Sdio4bit::new(
        &mut pio,
        dma.ch0,
        &timer,
        sm0,
        sm1,
        pin_sd_clk_id,
        pin_sd_cmd_id,
        pin_sd_dat0_id,
        1,
    );

    sd_controller.init().unwrap();

    // Initilaize the filesystem on the SD card

    let mut sd_mbr = MBR::new(sd_controller).unwrap();
    let sd_p1 = sd_mbr.get_partition(PartitionId::One).unwrap();

    let sd_fs = FileSystem::new(sd_p1, FsOptions::new()).unwrap();

    let sd_root = sd_fs.root_dir();

    // Create a name for the log file. Needs to not exist.
    // All files use the 8.3 DOS filename, as far as we are concerned
    let mut name_string = ArrayString::<12>::from_str("LOG").unwrap();

    for i in 0..u16::MAX {
        write!(&mut name_string, "{}.RDD", i).unwrap();

        if !sd_root.exists(&name_string).unwrap() {
            break;
        }

        name_string.truncate(3);
    }

    let mut log_file = sd_root.create_file(&name_string).unwrap();

    // ===================================================================== //
    // STEP 2, START CORE 1                                                  //
    // ===================================================================== //

    // First, alert the user to **NOT** touch the computer when core 1 is started.
    //
    // Core 1 will calculate it's heading and calibrate gravity based on the accelerations
    // read by the accelerometer so it's imperitive that it is not touched during
    // the calibration procedure.
    //
    // Blink 5 times giving a 5 second heads up to the end user.
    // Temporarily use a delay for coding ease of use, then immediately drop it.

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    for _ in 0..5 {
        pin_led.set_high().unwrap();
        delay.delay_ms(500);
        pin_led.set_low().unwrap();
        delay.delay_ms(500);
    }

    drop(delay);

    // Create the multicore instance

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);

    let cores = mc.cores();

    let core1 = &mut cores[1];

    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || main_1(i2c1))
        .unwrap();

    // Wait on the second core to ready up...

    let response = sio.fifo.read_blocking();

    match response {
        SIO_CMD_READY => {}
        SIO_CMD_PANIC => halt(), // If the other core has panic'd, do nothing
        _ => {
            panic!("Invalid response from core 1!");
        }
    }

    // Turn the LED solid to signify all is good!
    pin_led.set_high().unwrap();

    // ===================================================================== //
    // EXECUTION LOOP                                                        //
    // ===================================================================== //
    let mut commands_processed: u64 = 0;
    let mut loop_duration_us: u32 = 0; // Duration of the previous execution loop in us
    let mut data_left: usize = 0;
    let mut out_data: [u32; 22] = [0; 22];

    // Tell core 1 we are ready for data!
    sio.fifo.write_blocking(SIO_CMD_READY);

    loop {
        // Wait for core 1 to send an event
        cortex_m::asm::wfe();

        let start_time = timer.get_counter(); // Start time of this execution loop
                                              // Wait for commands from core 1
        if data_left == 0 {
            let command = sio.fifo.read_blocking();

            match command {
                SIO_CMD_PANIC => halt(),
                SIO_CMD_TELEM => {
                    let core_1_loop_duration_us = sio.fifo.read_blocking();
                    let time = sio.fifo.read_blocking();
                    let temperature_imu_c = sio.fifo.read_blocking();
                    let position_x_m = sio.fifo.read_blocking();
                    let position_y_m = sio.fifo.read_blocking();
                    let position_z_m = sio.fifo.read_blocking();
                    let heading_x_r = sio.fifo.read_blocking();
                    let heading_y_r = sio.fifo.read_blocking();
                    let heading_z_r = sio.fifo.read_blocking();
                    let velocity_x_ms = sio.fifo.read_blocking();
                    let velocity_y_ms = sio.fifo.read_blocking();
                    let velocity_z_ms = sio.fifo.read_blocking();
                    let rate_x_rs = sio.fifo.read_blocking();
                    let rate_y_rs = sio.fifo.read_blocking();
                    let rate_z_rs = sio.fifo.read_blocking();
                    let imu_accel_x_ms2 = sio.fifo.read_blocking();
                    let imu_accel_y_ms2 = sio.fifo.read_blocking();
                    let imu_accel_z_ms2 = sio.fifo.read_blocking();
                    let acceleration_x_ms2 = sio.fifo.read_blocking();
                    let acceleration_y_ms2 = sio.fifo.read_blocking();
                    let acceleration_z_ms2 = sio.fifo.read_blocking();

                    out_data = [
                        loop_duration_us,
                        core_1_loop_duration_us,
                        time,
                        temperature_imu_c,
                        position_x_m,
                        position_y_m,
                        position_z_m,
                        heading_x_r,
                        heading_y_r,
                        heading_z_r,
                        velocity_x_ms,
                        velocity_y_ms,
                        velocity_z_ms,
                        rate_x_rs,
                        rate_y_rs,
                        rate_z_rs,
                        imu_accel_x_ms2,
                        imu_accel_y_ms2,
                        imu_accel_z_ms2,
                        acceleration_x_ms2,
                        acceleration_y_ms2,
                        acceleration_z_ms2,
                    ];

                    data_left = (22 * 4) - log_file.write(&bytes_of(&out_data)).unwrap();
                }
                _ => {
                    panic!("Invalid response from core 1!");
                }
            }
        } else {
            data_left = data_left
                - log_file
                    .write(&bytes_of(&out_data)[(22 * 4) - data_left..])
                    .unwrap();
        }

        // Every 1000 commands flush the file
        commands_processed += 1;

        if (commands_processed % 1000) == 0 {
            log_file.flush().unwrap();
        }

        loop_duration_us = timer
            .get_counter()
            .checked_duration_since(start_time)
            .unwrap()
            .to_micros() as u32;
        // Tell core 1 we are ready for more data!
        if data_left == 0 {
            sio.fifo.write_blocking(SIO_CMD_READY);
        }
    }
}

/// Core 1 main function, called by core 0
///
/// Handles input from the IMU and keeps track of the rocket's position
///
/// Brain of the operation
///
/// Uses alarm 3 for timing
fn main_1(mut i2c1: I2C<I2C1, (Pin<Gpio6, FunctionI2C>, Pin<Gpio7, FunctionI2C>)>) -> ! {
    // ===================================================================== //
    // STEP 1, INITIALIZATION!                                               //
    // ===================================================================== //

    // !SAFETY! Core 0 has finished initialization, and the following I/Os
    // are exclusively allocated to core 1(supplied through the i2c argument):
    //
    //  * I2C1_SDA
    //  * I2C1_SCL

    let mut pac = unsafe { pac::Peripherals::steal() };
    let mut sio = hal::Sio::new(pac.SIO);

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let trig = TrigTableF32::new(&TRIG_TBL);
    let mut alarm_3 = timer.alarm_3().unwrap();

    // Check the IMU
    let mut response: [u8; 1] = [0; 1];
    // Remember you have to write to i2c to read from i2c...
    i2c1.write_read(IMU_ADDR, &[IMU_CHECK_REG], &mut response)
        .unwrap();

    if response[0] != IMU_CHECK_VAL {
        panic!("IMU NOT OK");
    }

    // Configure the IMU to have an initial sensitivity of +- 4g and a refresh
    // rate of 1.66kHz
    i2c1.write(IMU_ADDR, &IMU_CFG).unwrap();

    // ===================================================================== //
    // STEP 1.1, HEADING CALCULATION                                         //
    // ===================================================================== //

    // First we need to get a good average of the acceleration experienced by
    // each axis, about 1 second worth of readings

    let mut imu_accel_x_ms2: f32 = 0.0; // Acceleration in the X plane, in m/s^2. Relative to IMU
    let mut imu_accel_y_ms2: f32 = 0.0; // Acceleration in the Y plane, in m/s^2. Relative to IMU
    let mut imu_accel_z_ms2: f32 = 0.0; // Acceleration in the Z plane, in m/s^2. Relative to IMU

    const SAMPLE_COUNT: usize = 1000;
    for _ in 0..SAMPLE_COUNT {
        // Schedule the alarm
        //alarm_3.schedule(MicrosDurationU32::micros(1000)).unwrap();

        // Gather telemetry

        let mut read_buffer: [i16; 3] = [0; 3]; // All the registers needed to be read
                                                // occupy 14 bytes, 7 half-words

        let read_buffer_bytes = bytes_of_mut(&mut read_buffer);
        i2c1.write_read(IMU_ADDR, &[IMU_ACCEL_REG], read_buffer_bytes)
            .unwrap();

        // Move the registers into variables
        let imu_accel_x_raw = read_buffer[0];
        let imu_accel_y_raw = read_buffer[1];
        let imu_accel_z_raw = read_buffer[2];

        // Average out the newly aquired acclerations(using the default acceleration multiplier)
        imu_accel_x_ms2 +=
            (imu_accel_x_raw as f32) * IMU_ACCEL_MULT_4G * (1.0 / (SAMPLE_COUNT as f32));
        imu_accel_y_ms2 +=
            (imu_accel_y_raw as f32) * IMU_ACCEL_MULT_4G * (1.0 / (SAMPLE_COUNT as f32));
        imu_accel_z_ms2 +=
            (imu_accel_z_raw as f32) * IMU_ACCEL_MULT_4G * (1.0 / (SAMPLE_COUNT as f32));

        //if !alarm_3.finished() {cortex_m::asm::wfe();}
        //while !alarm_3.finished() {}
    }

    // Calculate the heading

    // For the X axis, Y is the opposite and Z is the adjacent
    let mut heading_x_r: f32 = atan2f(imu_accel_y_ms2, imu_accel_z_ms2);

    // For the Y axis, X is the -opposite and Z is the adjacent
    let mut heading_y_r: f32 = atan2f(-imu_accel_x_ms2, imu_accel_z_ms2);

    // For the Z axis, X is the opposite and Y is the adjacent
    let mut heading_z_r: f32 = atan2f(imu_accel_x_ms2, imu_accel_y_ms2);

    // !TODO! all euler math past this point it to be replaced, keep in mind!
    // Correct the initial accelerations
    // Negate heading
    let sin_x = trig.sin(-heading_x_r);
    let sin_y = trig.sin(-heading_y_r);
    let sin_z = trig.sin(-heading_z_r);

    let cos_x = trig.cos(-heading_x_r);
    let cos_y = trig.cos(-heading_y_r);
    let cos_z = trig.cos(-heading_z_r);

    // Acceleration across the X is affected by Y and Z heading
    let mut acceleration_x_ms2 =
        sin_y * imu_accel_z_ms2 + cos_y * (cos_z * imu_accel_x_ms2 - sin_z * imu_accel_y_ms2);

    // Acceleration across the Y is affected by X and Z heading
    let mut acceleration_y_ms2 =
        sin_z * imu_accel_x_ms2 + cos_z * (cos_x * imu_accel_y_ms2 - sin_x * imu_accel_z_ms2);

    // Acceleration across the Z is affected by X and Y heading
    let mut acceleration_z_ms2 =
        sin_x * imu_accel_y_ms2 + cos_x * (cos_y * imu_accel_z_ms2 - sin_y * imu_accel_x_ms2);

    // Calibrate to gravity
    let accel_adjust = GRAVITY_MS2 / acceleration_z_ms2;

    // Multiply all accelerations by accel adjust
    acceleration_x_ms2 *= accel_adjust;
    acceleration_y_ms2 *= accel_adjust;
    acceleration_z_ms2 *= accel_adjust;

    // Subtract gravity from the Z acceleration
    acceleration_z_ms2 -= GRAVITY_MS2;

    // ===================================================================== //
    // EXECUTION LOOP                                                        //
    // ===================================================================== //

    // Notify the first core we are good!
    sio.fifo.write_blocking(SIO_CMD_READY);

    // Telemetry variables
    let mut loop_duration_us: u32 = 0; // Duration of the previous execution loop in us
    let mut time: f32 = 0.0; // Time in seconds
    let mut temperature_imu_c: f32 = 0.0; // Temperature of IMU in C
    let mut position_x_m: f32 = 0.0; // Position in the X plane, in meters
    let mut position_y_m: f32 = 0.0; // Position in the Y plane, in meters
    let mut position_z_m: f32 = 0.0; // Position in the Z plane, in meters
    let mut velocity_x_ms: f32 = 0.0; // Velocity in the X plane, in m/s
    let mut velocity_y_ms: f32 = 0.0; // Velocity in the Y plane, in m/s
    let mut velocity_z_ms: f32 = 0.0; // Velocity in the Z plane, in m/s
    let mut rate_x_rs: f32 = 0.0; // Angular rate along the X axis in r/s
    let mut rate_y_rs: f32 = 0.0; // Angular rate along the Y axis in r/s
    let mut rate_z_rs: f32 = 0.0; // Angular rate along the Z axis in r/s

    // Math variables
    // With the default 4g range we have a sensitivity of 0.122 mg/LSB which
    // shall be converted to m/s^2
    //
    // 0.122e-3g -> 0.001196 m/s^2
    //
    // This means every LSB is equal to 1.196e-3m/s^2
    let imu_accel_multiplier: f32 = IMU_ACCEL_MULT_4G;
    let rate_multiplier: f32 = IMU_RATE_MULT_250DPS;
    let epoch = TimerInstantU64::from_ticks(0);

    loop {
        // Schedule the alarm
        alarm_3.schedule(MicrosDurationU32::micros(1000)).unwrap();
        let start_time = timer.get_counter(); // Start time of this execution loop
                                              // Send an event to core 0 to wake it up
        cortex_m::asm::sev();

        // Send everything to core 0 to be written to the SD card, if it is ready

        if let Some(cmd) = sio.fifo.read() {
            match cmd {
                SIO_CMD_READY => {
                    sio.fifo.write_blocking(SIO_CMD_TELEM);
                    sio.fifo.write_blocking(loop_duration_us);
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(time.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(temperature_imu_c.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(position_x_m.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(position_y_m.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(position_z_m.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(heading_x_r.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(heading_y_r.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(heading_z_r.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(velocity_x_ms.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(velocity_y_ms.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(velocity_z_ms.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(rate_x_rs.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(rate_y_rs.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(rate_z_rs.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(imu_accel_x_ms2.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(imu_accel_y_ms2.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(imu_accel_z_ms2.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(acceleration_x_ms2.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(acceleration_y_ms2.to_ne_bytes()));
                    sio.fifo
                        .write_blocking(u32::from_ne_bytes(acceleration_z_ms2.to_ne_bytes()));
                }
                SIO_CMD_PANIC => halt(),
                _ => {
                    panic!("Invalid response from core 0!");
                }
            }
        }
        // Gather telemetry

        let mut read_buffer: [i16; 7] = [0; 7]; // All the registers needed to be read
                                                // occupy 14 bytes, 7 half-words

        let read_buffer_bytes = bytes_of_mut(&mut read_buffer);
        i2c1.write_read(IMU_ADDR, &[IMU_TELM_REG], read_buffer_bytes)
            .unwrap();

        // Move the registers into variables
        let temperature_imu_raw = read_buffer[0];

        let rate_x_raw = read_buffer[1];
        let rate_y_raw = read_buffer[2];
        let rate_z_raw = read_buffer[3];

        let imu_accel_x_raw = read_buffer[4];
        let imu_accel_y_raw = read_buffer[5];
        let imu_accel_z_raw = read_buffer[6];

        // Convert the raw variables into the actual telemetry variables we need
        let prev_time = time;

        // Convert the micros to seconds
        time = ((start_time
            .checked_duration_since(epoch)
            .unwrap()
            .to_micros()) as f32)
            * 1e-6;

        temperature_imu_c = (temperature_imu_raw as f32) * IMU_TEMP_MULT;

        rate_x_rs = (rate_x_raw as f32) * rate_multiplier;
        rate_y_rs = (rate_y_raw as f32) * rate_multiplier;
        rate_z_rs = (rate_z_raw as f32) * rate_multiplier;

        imu_accel_x_ms2 = (imu_accel_x_raw as f32) * imu_accel_multiplier;
        imu_accel_y_ms2 = (imu_accel_y_raw as f32) * imu_accel_multiplier;
        imu_accel_z_ms2 = (imu_accel_z_raw as f32) * imu_accel_multiplier;

        // Compute everyting else
        let time_delta = time - prev_time;

        let heading_delta_x_r = rate_x_rs * time_delta;
        let heading_delta_y_r = rate_y_rs * time_delta;
        let heading_delta_z_r = rate_z_rs * time_delta;

        heading_x_r += heading_delta_x_r;
        heading_y_r += heading_delta_y_r;
        heading_z_r += heading_delta_z_r;

        // Rotate all the accelerations to match the coordinate system's axies
        let sin_x = trig.sin(-heading_x_r);
        let sin_y = trig.sin(-heading_y_r);
        let sin_z = trig.sin(-heading_z_r);

        let cos_x = trig.cos(-heading_x_r);
        let cos_y = trig.cos(-heading_y_r);
        let cos_z = trig.cos(-heading_z_r);

        // Acceleration across the X is affected by Y and Z heading
        acceleration_x_ms2 =
            sin_y * imu_accel_z_ms2 + cos_y * (cos_z * imu_accel_x_ms2 - sin_z * imu_accel_y_ms2);

        // Acceleration across the Y is affected by X and Z heading
        acceleration_y_ms2 =
            sin_z * imu_accel_x_ms2 + cos_z * (cos_x * imu_accel_y_ms2 - sin_x * imu_accel_z_ms2);

        // Acceleration across the Z is affected by X and Y heading
        acceleration_z_ms2 =
            sin_x * imu_accel_y_ms2 + cos_x * (cos_y * imu_accel_z_ms2 - sin_y * imu_accel_x_ms2);

        // Multiply all accelerations by accel adjust
        acceleration_x_ms2 *= accel_adjust;
        acceleration_y_ms2 *= accel_adjust;
        acceleration_z_ms2 *= accel_adjust;

        // Subtract gravity from the Z acceleration
        acceleration_z_ms2 -= GRAVITY_MS2;

        // Calculate new coordinates given accelerations
        // D = vt + 0.5at^2
        let time_delta_sqrd_hlvd = time_delta * time_delta * 0.5;
        position_x_m += velocity_x_ms * time_delta + acceleration_x_ms2 * time_delta_sqrd_hlvd;
        position_y_m += velocity_y_ms * time_delta + acceleration_y_ms2 * time_delta_sqrd_hlvd;
        position_z_m += velocity_z_ms * time_delta + acceleration_z_ms2 * time_delta_sqrd_hlvd;

        // Calculate new velocity given accelerations
        velocity_x_ms += acceleration_x_ms2 * time_delta;
        velocity_y_ms += acceleration_y_ms2 * time_delta;
        velocity_z_ms += acceleration_z_ms2 * time_delta;

        // Store the loop time to the previous time variable
        loop_duration_us = timer
            .get_counter()
            .checked_duration_since(start_time)
            .unwrap()
            .to_micros() as u32;
        // Wait for the sleep timer to finish
        if !alarm_3.finished() {
            cortex_m::asm::wfe();
        }
        while !alarm_3.finished() {}
    }
}

fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

static XMORSE: [u8; 256] = [
    67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 12, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 0, 35, 65, 98, 99, 100, 66, 24, 58, 59, 101, 102, 17, 36, 15,
    103, 104, 64, 105, 106, 107, 108, 109, 110, 111, 112, 62, 33, 113, 114, 115, 32, 116, 30, 37,
    48, 50, 42, 44, 40, 27, 28, 63, 45, 41, 39, 46, 34, 43, 53, 49, 38, 29, 60, 61, 31, 117, 51,
    118, 52, 119, 54, 120, 121, 122, 4, 23, 20, 11, 1, 19, 21, 7, 8, 56, 26, 10, 14, 6, 3, 22, 55,
    9, 5, 2, 13, 25, 18, 47, 16, 57, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134,
    135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153,
    154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
    173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
    192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
    211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
    230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
    249, 250, 251, 252, 253, 254, 255,
];

#[panic_handler]
fn panic(panic_info: &PanicInfo) -> ! {
    // Panicing uses Spinlock30

    let _lock = Spinlock30::claim();
    unsafe {
        let mut message_string = ArrayString::<256>::new();

        write!(&mut message_string, "{}", panic_info).unwrap_unchecked();

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

        let mut sio = hal::Sio::new(pac.SIO);

        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // Notify the other core we are panicing

        sio.fifo.write(SIO_CMD_PANIC);

        // Initialize GPIO
        let mut pin_led = pins.gpio25.into_push_pull_output();

        let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

        loop {
            // Flash it at 4hz
            for _ in 0..4 * 10 {
                pin_led.set_high().unwrap_unchecked();
                delay.delay_ms(125);
                pin_led.set_low().unwrap_unchecked();
                delay.delay_ms(125);
            }

            delay.delay_ms(1000);
            for byte in message.as_bytes() {
                let mut byte: u8 = XMORSE[*byte as usize];
                // Flash quick twice to indicate new byte
                pin_led.set_high().unwrap_unchecked();
                delay.delay_ms(125);
                pin_led.set_low().unwrap_unchecked();
                delay.delay_ms(125);
                pin_led.set_high().unwrap_unchecked();
                delay.delay_ms(125);
                pin_led.set_low().unwrap_unchecked();
                delay.delay_ms(1000);

                while byte > 0 {
                    let delaytime = match (byte & 1) > 0 {
                        true => 125,
                        false => 500,
                    };
                    pin_led.set_high().unwrap_unchecked();
                    delay.delay_ms(delaytime);
                    pin_led.set_low().unwrap_unchecked();
                    delay.delay_ms(1000 - delaytime);
                    byte >>= 1;
                }
            }
        }
    }
}
