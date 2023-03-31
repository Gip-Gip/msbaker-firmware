//! Code used to interface with SD cards using SDIO

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
// Ms. Baker's firmware(in a file named LEGAL-SOFTWARE).
// If not, see <https://www.gnu.org/licenses/>.

// You will see many expanded 32 bit integers for use with reading and setting
// flags and registers
//
// Here are the bit positions for two word commands
// 
// Word 0:
//  6666 5555 5555 5544  4444 4444 3333 3333
//  3210 9876 5432 1098  7654 3210 9876 5432
//  XXXX_XXXX_XXXX_XXXX__XXXX_XXXX_XXXX_XXXX
//
// Word 1:
//  3322 2222 2222 1111  1111 1100 0000 0000
//  1098 7654 3210 9876  5432 1098 7654 3210
//  XXXX_XXXX_XXXX_XXXX__XXXX_XXXX_XXXX_XXXX

use hal::pio::{MovStatusConfig, PinDir, ShiftDirection, PIOExt, PIO, PIOBuilder, SM0, SM1, StateMachine, UninitStateMachine, Running, Rx, Tx};
use pio_proc::pio_file;
use cortex_m::delay::Delay;
use rp2040_hal as hal;
use crate::errors::MsBakerError;

pub const SD_CMD_TIMEOUT_US: u32 = 1_000_000;

/// CRC7 Table used for calculating all 7-bit sd-card crcs
pub static CRC7_TABLE: [u8; 256] = [
    0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e, 0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
    0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c, 0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
    0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a, 0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
    0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28, 0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
    0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6, 0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
    0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84, 0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
    0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2, 0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
    0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
    0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc, 0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
    0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce, 0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
    0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98, 0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
    0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa, 0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
    0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34, 0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
    0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06, 0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
    0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50, 0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
    0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62, 0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2,
];

/// Simple function that calculates a crc7 from words.
/// skip specifies how many bytes to skip before the data begins, len specifies
/// the length of the data in bytes.
pub fn calculate_crc7_from_words(words: &[u32], skip: usize, len: usize) -> u8 {
    let mut crc: u8 = 0;

    let start_word = skip / 4; // Words are 32 bits long
    let start_shift = 24 - ((skip % 4) * 8);

    let mut word = start_word;
    let mut shift = start_shift;

    for _ in 0 .. len {
        // Calculate crc
        let byte = ((words[word] >> shift) & 0xFF) as u8;
        let index = byte ^ crc;
        crc = CRC7_TABLE[index as usize];

        // Advance the shift and/or the word

        if shift == 0 {
            shift = 24;
            word += 1;
        }
        else {
            shift -= 8;
        }
    }

    crc
}

/// Possible commands to give to the SD card
#[derive(PartialEq)]
pub enum SdCmd {
    /// CMD0: GO_IDLE_STATE, R0
    GoIdleState,
    /// CMD2: ALL_SEND_CID, R2.
    /// CMD3: SEND_RELATIVE_ADDR, R6.
    /// CMD6: SWITCH_FUNC, R1
    /// CMD7: SELECT_DESELECT_CARD, R1b.
    /// CMD8: SEND_IF_COND, asks if the sd card supports the current voltage along if
    /// it supports pcie. Supply the check pattern to be verified.
    SendIfCond(u8),
    /// CMD9: SEND_CSD, R1.
    /// CMD10: SEND_CID, R1.
    /// CMD12: STOP_TRANSMISSION, R1b.
    /// CMD13: SEND_STATUS, R2.
    /// CMD16: SET_BLOCKLEN, R1.
    /// CMD17: READ_SINGLE_BLOCK, R1.
    /// CMD18: READ_MULTIPLE_BLOCK, R1.
    /// CMD24: WRITE_BLOCK, R1.
    /// CMD25: WRITE_MULTIPLE_BLOCK, R1.
    /// CMD27: PROGRAM_CSD, R1.
    /// CMD32: ERASE_WR_BLK_START, R1.
    /// CMD33: ERASE_WR_BLK_END, R1.
    /// CMD38: ERASE, R1b.
    /// CMD55: APP_CMD, R1. **DO NOT USE.** APP_CMD is automatically sent before any ACMD.
    /// Supply the RCA with this command
    AppCmd(u16),
    /// CMD56: GEN_CMD
    /// ACMD6: SET_BUS_WIDTH
    /// ACMD13: SD_STATUS
    /// ACMD22: SEND_NUM_WR_BLOCKS
    /// ACMD23: SET_WR_BLK_ERASE_COUNT
    /// ACMD41: SD_APP_OP_COND, Sends host capacity support info and asks the card to send the OCR
    /// Supply HCS, XPC, S18R, and Voltage Window
    SdAppOpCond(bool, bool, bool, u32),
    /// ACMD42: SET_CLR_CARD_DETECT
    /// ACMD51: SEND_SCR
}

impl SdCmd {
    /// Get the index of a command
    pub fn get_cmd_index(&self) -> u8 {
        match self {
            SdCmd::GoIdleState => 0,
            SdCmd::SendIfCond(_) => 8,
            SdCmd::SdAppOpCond(_, _, _, _) => 41,
            SdCmd::SendScr => 51,
            SdCmd::AppCmd(_) => 55,
        }
    }

    /// Get the response type of a command
    pub fn get_cmd_response(&self) -> SdCmdResponseType {
        match self {
            SdCmd::GoIdleState => SdCmdResponseType::R0,
            SdCmd::SendScr |
                SdCmd::AppCmd(_) => SdCmdResponseType::R1,
            SdCmd::SdAppOpCond(_, _, _, _) => SdCmdResponseType::R3,
            SdCmd::SendIfCond(_) => SdCmdResponseType::R7,
        }
    }

    /// Returns true if a command is an app command
    pub fn is_acmd(&self) -> bool {
        match self {
            SdCmd::GoIdleState |
                SdCmd::SendIfCond(_) |
                SdCmd::AppCmd(_) => false,
            SdCmd::SdAppOpCond(_, _, _, _) => true,
        }
    }

    /// Format command as 2 words. For use with send_command
    pub fn format(&self) -> [u32; 2] {
        // Bits 63-56 are the length of the command in bits, -1
        // Always 47, but specified because PIO asm can't autonomously set it's
        // internal registers to anything above 31
        //
        // Bits 55-54 will always be 0b01, as it specifies the starting bit(0) and the command
        // direction(host->card)
        //
        // Bit 8 is always 1(end bit)
        let mut data: [u32; 2] = [
            0b0010_1111_0100_0000__0000_0000_0000_0000,
            0b0000_0000_0000_0000__0000_0001_0000_0000,
        ];

        // Bits 53-48 is the command index
        data[0] |= (self.get_cmd_index() as u32) << 16;

        // Bits 47-16 is the argument
        match self {
            Self::SendIfCond(test_pattern) => {
                // Bits 47-28 are reserved
                // 27-24 is the supply voltage, however always 0b0001 (2.7-3.6v)
                // both because that is the rp2040 voltage and because no other
                // ranges are specified
                data[1] |= 0b0000_0001_0000_0000__0000_0000_0000_0000;
                // 23-16 is the check pattern
                data[1] |= (*test_pattern as u32) << 16;
            }
            Self::SdAppOpCond(hcs, xpc, s18r, voltage_window) => {
                // Bit 47 is reserved
                // Bit 46 is the HCS
                if hcs {
                    data[0] |= 0b0000_0000_0000_0000__0100_0000_0000_0000;
                }
                // Bit 45 is reserved
                // Bit 44 is XPC
                if xpc {
                    data[0] |= 0b0000_0000_0000_0000__0001_0000_0000_0000;
                }
                // Bits 43-41 are reserved
                // Bit 40 is S18R
                if s18r {
                    data[0] |= 0b0000_0000_0000_0000__0000_0001_0000_0000;
                }

                // Bits 39-16 is the Voltage Window
                data[0] |= (voltage_window >> 8) & 0xFF;
                data[1] |= (voltage_window << 16) & 0xFFFF_0000;
            }
            Self::AppCmd(rca) => {
                // Bits 47-32 are the RCA, 31-16 are stuff bits
                data[0] |= *rca as u32;
            }
            // And some commands don't have an argument
            _ => {}
        }

        // Bits 15-9 is the crc7 of the command, processed in big endian
        let mut crc = calculate_crc7_from_words(&data, 1, 5);
        data[1] |= (crc as u32) << 8;

        // Bits 7-0 is the length of the response to the command - 1
        let response_len = self.get_cmd_response().get_response_len() as u32;

        if response_len > 0 {
            data[1] |= response_len - 1;
        }

        data
    }
}

/// Possible response types for SD commands
#[derive(PartialEq)]
pub enum SdCmdResponseType {
    /// R0, no response
    R0,
    /// R1, normal response
    R1,
    /// R1b, normal response with busy
    R1b,
    /// R2, CID/CSD register
    R2,
    /// R3, OCR register
    R3,
    /// R6, Published RCA response
    R6,
    /// R7, Card interface condition
    R7,
}

impl SdCmdResponseType {
    /// Get the length of the response in bits
    pub fn get_response_len(&self) -> u8 {
        match self {
            SdCmdResponseType::R0 => 0,
            SdCmdResponseType::R1
            | SdCmdResponseType::R1b
            | SdCmdResponseType::R3
            | SdCmdResponseType::R6
            | SdCmdResponseType::R7 => 48,
            SdCmdResponseType::R2 => 136,
        }
    }
}

/// Same as SdCmdResponseType, just for returning the actual values
pub enum SdCmdResponse {
    /// R0, returns nothing
    R0,
    /// R1, returns the card status
    R1,
    /// R1b, identical to R1
    R1b,
    /// R2, returns cid or csd
    R2,
    /// R3, returns OCR register
    R3,
    /// R6, returns RCA
    R6,
    /// R7, returns CIC
    R7(SdCic),
}

pub struct SdCic {
    /// Set to true if the card 
    supports_1p2v: bool,
    /// Set to true if the card supports pcie
    supports_pcie: bool,
}

/// SDIO 4bit interface struct
pub struct Sdio4bit<P: PIOExt> {
    /// PIO struct, can be PIO0 or PIO1. programs are installed to this.
    pio: PIO<P>,
    /// Reference to the processor's delay struct, used for timing purposes
    delay: Delay,
    /// The command state machine, controls SD_CLK and SD_CMD
    sm_cmd: StateMachine<(P, SM0), Running>,
    sm_cmd_rx: Rx<(P, SM0)>,
    sm_cmd_tx: Tx<(P, SM0)>,
    // The data state machine, controls SD_DAT0-SD_DAT3
    // sm_dat: StateMachine<(P, SM1), Running>,
    // sm_dat_rx: Rx<(P, SM1)>,
    // sm_dat_tx: Tx<(P, SM1)>,
}

impl<P: PIOExt> Sdio4bit<P> {
    pub fn new(
            mut pio: PIO<P>,
            mut delay: Delay,
            sm0: UninitStateMachine<(P, SM0)>,
            sm1: UninitStateMachine<(P, SM1)>,
            sd_clk_id: u8,
            sd_cmd_id: u8,
            sd_dat0_id: u8,
            sd_dat1_id: u8,
            sd_dat2_id: u8,
            sd_dat3_id: u8) -> Self {
        // Initialilze the raw program variables from the rp2040_sdio.pio file
        let program_sdio_cmd_clk =
            pio_file!("src/rp2040_sdio.pio", select_program("sdio_cmd_clk")).program;
        // let program_sdio_data_rx =
        //     pio_file!("src/rp2040_sdio.pio", select_program("sdio_data_rx")).program;
        // let program_sdio_data_tx =
        //     pio_file!("src/rp2040_sdio.pio", select_program("sdio_data_tx")).program;

        // Install them to the pio
        let program_sdio_cmd_clk = pio.install(&program_sdio_cmd_clk).unwrap();
        // let program_sdio_data_rx = pio.install(&program_sdio_data_rx).unwrap();
        // let program_sdio_data_tx = pio.install(&program_sdio_data_tx).unwrap();

        let (mut sm_cmd, sm_cmd_rx, sm_cmd_tx) = PIOBuilder::from_program(program_sdio_cmd_clk)
            .set_mov_status_config(MovStatusConfig::Tx(2))
            .set_pins(sd_cmd_id, 1)
            .out_pins(sd_cmd_id, 1)
            .in_pin_base(sd_cmd_id)
            .jmp_pin(sd_cmd_id)
            .side_set_pin_base(sd_clk_id)
            .out_shift_direction(ShiftDirection::Left)
            .in_shift_direction(ShiftDirection::Left)
            .clock_divisor_fixed_point(25, 0)
            .autopush(true)
            .autopull(true)
            .build(sm0);

        sm_cmd.set_pindirs([(sd_clk_id, PinDir::Output), (sd_cmd_id, PinDir::Output)]);
        // Start the state machine
        let sm_cmd = sm_cmd.start();

        Self {
            pio,
            delay,
            sm_cmd,
            sm_cmd_rx,
            sm_cmd_tx,
        }
    }

    /// Initialize the SD card
    pub fn init(&mut self) -> Result<(), MsBakerError> {
        // Wait 1ms to ensure that the card is properly initialized
        self.delay.delay_ms(1);

        // Send CMD0
        self.send_command(SdCmd::GoIdleState)?;

        // Send CMD8
        let cic = match self.send_command(SdCmd::SendIfCond(0xAA))? {
            SdCmdResponse::R7(cic) => cic,
            _ => {panic!("INVALID RESPONSE TYPE(PE)");}
        };

        Ok(())
    }

    pub fn send_command(&mut self, command: SdCmd) -> Result<SdCmdResponse, MsBakerError> {
        // If the command is an app command, first send the AppCmd command
        if command.is_acmd() {
            self.send_command(SdCmd::AppCmd(0x0000));
        }

        // Get the data for the command
        let command_data = command.format();

        // Push both words into tx
        self.sm_cmd_tx.write(command_data[0]);
        self.sm_cmd_tx.write(command_data[1]);

        let response_type = command.get_cmd_response();

        // Create a buffer and make the actually accessable portion equal to
        // the response size. Max size is R2, which is 136 bits/17 bytes/4.25 wqrds
        let mut resp_buf: [u32; 5] = [0; 5];

        // Calculate the length of the response, and add 1 since all response lengths are not
        // divisable by 32 bits
        let resp_len_bits = response_type.get_response_len();
        let mut resp_len: usize = ((resp_len_bits / 32) + 1) as usize;

        for i in 0..resp_len {
            // Wait until there is something in the fifo
            // !TODO! Implement better timeout detection
            let mut timeout_us = SD_CMD_TIMEOUT_US;
            while self.sm_cmd_rx.is_empty() {
                if timeout_us == 0 {
                    return Err(MsBakerError::SdioCmdTimeout{cmd: command.get_cmd_index(), timeout_time: SD_CMD_TIMEOUT_US});
                }
                self.delay.delay_us(1);
                timeout_us -= 1;
            }

            resp_buf[i] = self.sm_cmd_rx.read().unwrap();
        }

        let crc = (resp_buf[1] & 0xFE) as u8;
        // shift the last word so the same crc calculation can be used for in
        // and out data
        resp_buf[resp_len - 1] = resp_buf[resp_len - 1] << (32 - (resp_len_bits % 32));

        // Return here if no response, otherwise get the response
        if response_type == SdCmdResponseType::R0 {
            return Ok(SdCmdResponse::R0);
        }

        // Verifty the crc
        let good_crc = calculate_crc7_from_words(&resp_buf, 0, (resp_len_bits / 8) as usize );

        let crc = ((resp_buf[1] >> 16) & 0xFE) as u8;

        if good_crc != crc {
            return Err(MsBakerError::SdioBadCrc7{good_crc, bad_crc: crc});
        }

        match response_type {
            SdCmdResponseType::R7 => {
                if let SdCmd::SendIfCond(good_check_pattern) = command {
                // Bit 21 is the 1.2v support bit, 20 is the pcie support
                //                                   4444 4444 3333 3333  3322 2222 2222 1111
                //                                   7654 3210 9876 5432  1098 7654 3210 9876
                let supports_1p2v = (resp_buf[0] & 0x0000_0000_0000_0000__0000_0000_0010_0000) > 0;
                let supports_pcie = (resp_buf[0] & 0x0000_0000_0000_0000__0000_0000_0001_0000) > 0;
                
                // Verify the correct voltage is in use
                let voltage = (resp_buf[0] & 0x0000_0000_0000_0000__0000_0000_0000_1111) as u8;

                if voltage != 0b0001 {
                    return Err(MsBakerError::SdioBadVoltage{bad_volt : voltage});
                }

                // Verify the check pattern
                let check_pattern = ((resp_buf[1] >> 24) & 0xFF) as u8;

                if check_pattern != good_check_pattern {
                    return Err(MsBakerError::SdioBadCheck{good_check: good_check_pattern, bad_check: check_pattern});
                }


                Ok(SdCmdResponse::R7(SdCic {
                    supports_1p2v,
                    supports_pcie,
                }))
                }
                else {
                    // Shouldn't happen, but just incase
                    panic!("IMPROPER USE OF RESPONSE TYPE 7");
                }
            }
            _ => {
                todo!()
            }
        }
    }
}
