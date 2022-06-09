mod util;

use crate::canbus::CANBitrate;
use crate::slcan::util::{concat, BoundedU8};
use heapless;
use hex;
use packed_struct::prelude::*;

#[derive(Debug)]
pub enum SLCANError {
    Regular(ErrorKind),
}

pub fn err_queue_full(_e: u8) -> SLCANError {
    SLCANError::Regular(ErrorKind::QueueFull)
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ErrorKind {
    QueueFull,
    InvalidCommand,
    NotImplemented,
}

#[derive(Debug, Clone)]
pub struct QueueFullError;

#[derive(Debug, Clone)]
pub struct InvalidCommandError;

pub const COMMAND_TERMINATOR: u8 = b'\r';
pub const ERROR_CHAR: u8 = 7;

pub type QueueType = heapless::Deque<u8, 32>;
pub type CommandQueueType = heapless::Deque<Command, 8>;

trait HexOutput<const N: usize> {
    fn as_bytes(&self) -> [u8; N];

    fn as_hex(&self) -> [u8; N * 2] {
        let mut hex_str = [0u8; N * 2];
        hex::encode_to_slice(self.as_bytes(), &mut hex_str).unwrap();
        hex_str.make_ascii_uppercase();
        return hex_str;
    }
}

#[derive(PackedStruct)]
#[packed_struct(bit_numbering = "msb0")]
struct StatusFlags {
    #[packed_field(bits = "0")]
    receive_queue_full: bool,
    transmit_queue_full: bool,
    error_warning: bool,
    data_overrun: bool,
    #[packed_field(bits = "5")]
    error_passive: bool,
    arbitration_lost: bool,
    bus_error: bool,
}

impl StatusFlags {
    fn new() -> Self {
        StatusFlags {
            receive_queue_full: false,
            transmit_queue_full: false,
            error_warning: false,
            data_overrun: false,
            error_passive: false,
            arbitration_lost: false,
            bus_error: false,
        }
    }
}

impl HexOutput<1> for StatusFlags {
    fn as_bytes(&self) -> [u8; 1] {
        return self.pack().unwrap();
    }
}

enum CANChannelState {
    Open,
    Closed,
}

struct VersionInfo {
    hardware_version: u8,
    software_version: u8,
}

impl HexOutput<2> for VersionInfo {
    fn as_bytes(&self) -> [u8; 2] {
        return [self.hardware_version, self.software_version];
    }
}

pub struct SLCAN {
    command_queue: CommandQueueType,
    channel_state: CANChannelState,
    bitrate: Option<CANBitrate>,
    timestamps_enabled: bool,
    status: StatusFlags,
    version: VersionInfo,
    serial_number: [u8; 4],
}

impl SLCAN {
    pub fn new() -> Self {
        SLCAN {
            command_queue: CommandQueueType::new(),
            channel_state: CANChannelState::Closed,
            bitrate: None,
            timestamps_enabled: false,
            status: StatusFlags::new(),
            version: VersionInfo {
                hardware_version: 0xFA,
                software_version: 0x01,
            },
            serial_number: *b"F446",
        }
    }

    /// Handles a single received byte, pushing it to the rx queue.
    /// If a complete command has been received, returns it.
    pub fn handle_incoming_byte(
        &mut self,
        incoming_byte: u8,
        rx_queue: &mut QueueType,
    ) -> Result<Option<Command>, SLCANError> {
        let result = self.do_handle_incoming_byte(incoming_byte, rx_queue);
        if result.is_err() {
            self.status.receive_queue_full = true;
        }
        return result;
    }

    fn do_handle_incoming_byte(
        &mut self,
        incoming_byte: u8,
        rx_queue: &mut QueueType,
    ) -> Result<Option<Command>, SLCANError> {
        // If we received a command terminator, attempt to parse the rx queue as a single command
        if incoming_byte == COMMAND_TERMINATOR {
            let received_bytes: RequestData = rx_queue.iter().map(|&x| x).collect();
            rx_queue.clear();
            let command = Command::from_bytes(&received_bytes);
            return Some(command).transpose();
        }

        // Otherwise, just push to the queue
        rx_queue.push_front(incoming_byte).map_err(err_queue_full)?;
        return Ok(None);
    }

    /// Handles the outputs of a command, pushing to the tx queue.
    pub fn handle_command_output(
        &mut self,
        output: &CommandReturnType,
        tx_queue: &mut QueueType,
    ) -> Result<(), SLCANError> {
        let result = self
            .do_handle_command_output(output, tx_queue)
            .map_err(err_queue_full);
        if result.is_err() {
            self.status.transmit_queue_full = true;
        }
        return result;
    }

    fn do_handle_command_output(
        &mut self,
        output: &CommandReturnType,
        tx_queue: &mut QueueType,
    ) -> Result<(), u8> {
        match output {
            Ok(data) => {
                for b in data.iter() {
                    tx_queue.push_front(*b)?;
                }
                tx_queue.push_front(crate::slcan::COMMAND_TERMINATOR)?;
            }
            Err(_e) => {
                tx_queue.push_front(crate::slcan::ERROR_CHAR)?;
            }
        }
        Ok(())
    }
}

type LengthType = BoundedU8<0, 8>;

enum CommandVariant {
    Setup,
    SetupWithBTR,
    OpenChannel,
    CloseChannel,
    TransmitFrame,
    TransmitExtendedFrame,
    TransmitRTRFrame,
    TransmitExtendedRTRFrame,
    ReadStatusFlags,
    SetAcceptanceCode,
    SetAcceptanceMask,
    GetVersion,
    GetSerialNumber,
    EnableTimeStamps,
}

/// Data container for an SLCAN command
pub struct Command {
    variant: CommandVariant,
    data: heapless::Vec<u8, 12>,
}

type RequestData = heapless::Vec<u8, 32>;
pub type ResponseData = heapless::Vec<u8, 12>;
pub type CommandReturnType = Result<ResponseData, SLCANError>;

impl Command {
    /// Parses a Command from bytes. Variant identifier is checked here,
    /// but arguments are not checked until runtime.
    pub fn from_bytes(bytes: &RequestData) -> Result<Self, SLCANError> {
        let variant = match bytes.first() {
            Some(b'S') => CommandVariant::Setup,
            Some(b's') => CommandVariant::SetupWithBTR,
            Some(b'O') => CommandVariant::OpenChannel,
            Some(b'C') => CommandVariant::CloseChannel,
            Some(b't') => CommandVariant::TransmitFrame,
            Some(b'T') => CommandVariant::TransmitExtendedFrame,
            Some(b'r') => CommandVariant::TransmitRTRFrame,
            Some(b'R') => CommandVariant::TransmitExtendedRTRFrame,
            Some(b'F') => CommandVariant::ReadStatusFlags,
            Some(b'M') => CommandVariant::SetAcceptanceCode,
            Some(b'm') => CommandVariant::SetAcceptanceMask,
            Some(b'V') => CommandVariant::GetVersion,
            Some(b'N') => CommandVariant::GetSerialNumber,
            Some(b'Z') => CommandVariant::EnableTimeStamps,
            _ => return Err(SLCANError::Regular(ErrorKind::InvalidCommand)),
        };
        let data = heapless::Vec::from_slice(&bytes[1..])
            .map_err(|_e| SLCANError::Regular(ErrorKind::InvalidCommand))?;

        return Ok(Command {
            variant: variant,
            data: data,
        });
    }

    /// Runs the command, returning any bytes to be sent back over serial.
    pub fn run(&self, slcan: &mut SLCAN) -> CommandReturnType {
        match self.variant {
            CommandVariant::Setup => self.run_setup(slcan),
            CommandVariant::SetupWithBTR => self.run_not_implemented(slcan),
            CommandVariant::OpenChannel => self.run_open_channel(slcan),
            CommandVariant::CloseChannel => self.run_close_channel(slcan),
            CommandVariant::TransmitFrame => self.run_transmit_frame(slcan),
            CommandVariant::TransmitExtendedFrame => self.run_transmit_extended_frame(slcan),
            CommandVariant::TransmitRTRFrame => self.run_not_implemented(slcan),
            CommandVariant::TransmitExtendedRTRFrame => self.run_not_implemented(slcan),
            CommandVariant::ReadStatusFlags => self.run_read_status_flags(slcan),
            CommandVariant::SetAcceptanceCode => self.run_not_implemented(slcan),
            CommandVariant::SetAcceptanceMask => self.run_not_implemented(slcan),
            CommandVariant::GetVersion => self.run_get_version(slcan),
            CommandVariant::GetSerialNumber => self.run_get_serial_number(slcan),
            CommandVariant::EnableTimeStamps => self.run_enable_timestamps(slcan),
        }
    }

    /// Placeholder command for unimplemented commands
    fn run_not_implemented(&self, _slcan: &mut SLCAN) -> CommandReturnType {
        Err(SLCANError::Regular(ErrorKind::NotImplemented))
    }

    fn run_setup(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // do some setup things
        Ok(ResponseData::new())
    }

    fn run_open_channel(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // open the CAN channel
        Ok(ResponseData::new())
    }

    fn run_close_channel(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // close the CAN channel
        Ok(ResponseData::new())
    }

    fn run_transmit_frame(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // transmit a frame
        Ok(ResponseData::new())
    }

    fn run_transmit_extended_frame(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // transmit an extended frame
        Ok(ResponseData::new())
    }

    fn run_read_status_flags(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // return status flags
        Ok(ResponseData::from_slice(&concat(b"F", &slcan.status.as_hex())).unwrap())
    }

    fn run_get_version(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // return version
        Ok(ResponseData::from_slice(&concat(b"V", &slcan.version.as_hex())).unwrap())
    }

    fn run_get_serial_number(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // return serial number
        Ok(ResponseData::from_slice(&concat(b"N", &slcan.serial_number)).unwrap())
    }

    fn run_enable_timestamps(&self, slcan: &mut SLCAN) -> CommandReturnType {
        // set timestamps on or off
        match self.data.get(0) {
            Some(b'0') => {
                slcan.timestamps_enabled = false;
                return Ok(ResponseData::new());
            }
            Some(b'1') => {
                slcan.timestamps_enabled = true;
                return Ok(ResponseData::new());
            }
            _ => return Err(SLCANError::Regular(ErrorKind::InvalidCommand)),
        }
    }
}

struct Setup {
    can_bitrate: CANBitrate,
}

struct SetupWithBTR {
    btr0: u8,
    btr1: u8,
}

struct OpenChannel;

struct CloseChannel;

struct TransmitFrame {
    id: StandardID,
    data: heapless::Vec<u8, 8>,
}

struct TransmitExtendedFrame {
    id: ExtendedID,
    data: heapless::Vec<u8, 8>,
}

struct TransmitRTRFrame {
    id: StandardID,
    length: LengthType,
}

struct TransmitExtendedRTRFrame {
    id: ExtendedID,
    length: LengthType,
}

struct ReadStatusFlags;

struct SetAcceptanceCode {
    acceptance_code: heapless::Vec<u8, 4>,
}

struct SetAcceptanceMask {
    acceptance_mask: heapless::Vec<u8, 4>,
}

struct GetVersion;

struct GetSerialNumber;

struct EnableTimeStamps {
    enable_timestamps: bool,
}

// ****** end of command structs ******

struct StandardID {
    // 11-bit identifier
    id: u16,
}

struct ExtendedID {
    // 29-bit identifier
    id: u32,
}
