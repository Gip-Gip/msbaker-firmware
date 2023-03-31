//! Errors in the msbaker ecosystem

use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum MsBakerError {
    #[snafu(display("Bad CRC7! good_crc={good_crc}, bad_crc={bad_crc}"))]
    SdioBadCrc7 {good_crc: u8, bad_crc: u8},
    #[snafu(display("Timeout on SD command! cmd={cmd}, timeout_time={timeout_time}"))]
    SdioCmdTimeout {cmd: u8, timeout_time: u32},
    #[snafu(display("Bad CMD8 check pattern! good_check={good_check}, bad_check={bad_check}"))]
    SdioBadCheck {good_check: u8, bad_check: u8},
    #[snafu(display("Bad CMD8 voltage! good_volt=1, bad_volt={bad_volt}"))]
    SdioBadVoltage {bad_volt: u8},
}
