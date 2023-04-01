//! Errors in the msbaker ecosystem

use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum MsBakerError {
    #[snafu(display("(SDIO) Timeout on SD command! cmd={cmd}, timeout_time={timeout_time}"))]
    SdioCmdTimeout {cmd: u8, timeout_time: u32},
    #[snafu(display("(SDIO) Response is to the wrong command! good_cmd={good_cmd}, bad_cmd={bad_cmd}"))]
    SdioWrongCmd {good_cmd: u8, bad_cmd: u8},
    #[snafu(display("(SDIO) Bad CRC7! good_crc={good_crc}, bad_crc={bad_crc}"))]
    SdioBadCrc7 {good_crc: u8, bad_crc: u8},
    #[snafu(display("(SDIO) Bad CMD8 check pattern! good_check={good_check}, bad_check={bad_check}"))]
    SdioBadCheck {good_check: u8, bad_check: u8},
    #[snafu(display("(SDIO) Bad CMD8 voltage! good_volt=1, bad_volt={bad_volt}"))]
    SdioBadVoltage {bad_volt: u8},
    #[snafu(display("(SDIO) Bad OCR voltage range!"))]
    SdioBadVoltRange {},
    #[snafu(display("Programmer Error! Should definitely not happen!"))]
    PE {},
}
