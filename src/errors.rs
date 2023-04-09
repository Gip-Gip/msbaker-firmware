//! Errors in the msbaker ecosystem

use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum MsBakerError {
    #[snafu(display("(SDIO) Timeout on SD command!"))]
    SdioCmdTimeout {cmd: u8, timeout_time: u32},
    #[snafu(display("(SDIO) Timeout on write!"))]
    SdioWriteTimeout {},
    #[snafu(display("(SDIO) Response is to the wrong command!"))]
    SdioWrongCmd {good_cmd: u8, bad_cmd: u8},
    #[snafu(display("(SDIO) Bad rx CRC7!"))]
    SdioBadRxCrc7 {},
    #[snafu(display("(SDIO) Bad tx CRC7!"))]
    SdioBadTxCrc7 {},
    #[snafu(display("(SDIO) Bad tx CRC16!"))]
    SdioBadTxCrc16 {},
    #[snafu(display("(SDIO) Bad CMD8 check pattern!"))]
    SdioBadCheck {good_check: u8, bad_check: u8},
    #[snafu(display("(SDIO) Bad CMD8 voltage!"))]
    SdioBadVoltage {bad_volt: u8},
    #[snafu(display("(SDIO) Bad OCR voltage range!"))]
    SdioBadVoltRange {},
    #[snafu(display("(SDIO) Failed to write!"))]
    SdioWriteFail {},
    #[snafu(display("(SDIO) Unknown response to write!"))]
    SdioWriteUnknown {},
    #[snafu(display("(PE)SDIO already transferring/recieving! Should definitely not happen!"))]
    SdioInTxRx {},
    #[snafu(display("Programmer Error!"))]
    PE {},
}
