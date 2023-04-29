//! Errors in the msbaker ecosystem

use embedded_io::{blocking::ReadExactError, Error, ErrorKind};
use rp_sdio::errors::SdioError;
use snafu::prelude::*;

#[derive(Debug, Snafu)]
pub enum MsBakerError {
    #[snafu(display("SDIO Error!"))]
    Sdio { e: SdioError },
}

impl Error for MsBakerError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl From<SdioError> for MsBakerError {
    fn from(e: SdioError) -> Self {
        Self::Sdio { e }
    }
}
