mod util;

use crate::slcan::util::BoundedU8;
use heapless;
use packed_struct::prelude::*;

#[derive(Debug)]
pub enum SLCANError {
    Regular(ErrorKind),
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ErrorKind {
    QueueFull,
    InvalidCommand,
}

#[derive(Debug, Clone)]
pub struct QueueFullError;

#[derive(Debug, Clone)]
pub struct InvalidCommandError;

pub const COMMAND_TERMINATOR: u8 = b'\r';

pub type QueueType = heapless::Deque<u8, 32>;
pub type CommandQueueType = heapless::Deque<Command, 8>;

#[derive(PackedStruct)]
#[packed_struct(bit_numbering = "msb0")]
struct OutputStatusFlags {
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

enum CANChannelState {
    Open,
    Closed,
}

struct VersionInfo {
    hardware_version: u16,
    software_version: u16,
}

pub struct SLCAN {
    command_queue: CommandQueueType,
    channel_state: CANChannelState,
    bitrate: Option<CANBitrate>,
    timestamps_enabled: bool,
    version: VersionInfo,
    serial_number: u32,
}

impl SLCAN {
    pub fn new() -> Self {
        SLCAN {
            command_queue: CommandQueueType::new(),
            channel_state: CANChannelState::Closed,
            bitrate: None,
            timestamps_enabled: false,
            version: VersionInfo {
                hardware_version: 0xFA,
                software_version: 0x01,
            },
            serial_number: 0xF446,
        }
    }

    /// Handles a single received byte, pushing it to the rx queue.
    /// If a complete command has been received, returns it.
    pub fn handle_incoming_byte(
        &mut self,
        incoming_byte: u8,
        rx_queue: &mut QueueType,
    ) -> Result<Option<Command>, SLCANError> {
        // If we received a command terminator, attempt to parse the rx queue as a single command
        if incoming_byte == COMMAND_TERMINATOR {
            let received_bytes: RequestData = rx_queue.iter().map(|&x| x).collect();
            let command = Command::from_bytes(&received_bytes);
            return Some(command).transpose();
        }

        // Otherwise, just push to the queue
        if rx_queue.is_full() {
            return Err(SLCANError::Regular(ErrorKind::QueueFull));
        }

        rx_queue.push_front(incoming_byte).unwrap();
        return Ok(None);
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

/// Generic representation of any SLCAN command.
/// I really wanted to use polymorphism, but couldn't
/// find a way to do it without `alloc`.
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
        let data = heapless::Vec::from_slice(&bytes[1..]).unwrap();

        return Ok(Command {
            variant: variant,
            data: data,
        });
    }

    /// Runs the command, returning any bytes to be sent back over serial.
    pub fn run(&self) -> CommandReturnType {
        match self.variant {
            CommandVariant::Setup => self.run_setup(),
            CommandVariant::SetupWithBTR => self.run_not_implemented(),
            CommandVariant::OpenChannel => self.run_open_channel(),
            CommandVariant::CloseChannel => self.run_close_channel(),
            CommandVariant::TransmitFrame => self.run_transmit_frame(),
            CommandVariant::TransmitExtendedFrame => self.run_transmit_extended_frame(),
            CommandVariant::TransmitRTRFrame => self.run_not_implemented(),
            CommandVariant::TransmitExtendedRTRFrame => self.run_not_implemented(),
            CommandVariant::ReadStatusFlags => self.run_read_status_flags(),
            CommandVariant::SetAcceptanceCode => self.run_not_implemented(),
            CommandVariant::SetAcceptanceMask => self.run_not_implemented(),
            CommandVariant::GetVersion => self.run_get_version(),
            CommandVariant::GetSerialNumber => self.run_get_serial_number(),
            CommandVariant::EnableTimeStamps => self.run_enable_timestamps(),
        }
    }

    /// Placeholder command for unimplemented commands
    fn run_not_implemented(&self) -> CommandReturnType {
        Ok(ResponseData::new())
    }

    fn run_setup(&self) -> CommandReturnType {
        // do some setup things
        Ok(ResponseData::new())
    }

    fn run_open_channel(&self) -> CommandReturnType {
        // open the CAN channel
        Ok(ResponseData::new())
    }

    fn run_close_channel(&self) -> CommandReturnType {
        // close the CAN channel
        Ok(ResponseData::new())
    }

    fn run_transmit_frame(&self) -> CommandReturnType {
        // transmit a frame
        Ok(ResponseData::new())
    }

    fn run_transmit_extended_frame(&self) -> CommandReturnType {
        // transmit an extended frame
        Ok(ResponseData::new())
    }

    fn run_read_status_flags(&self) -> CommandReturnType {
        // return status flags
        Ok(ResponseData::new())
    }

    fn run_get_version(&self) -> CommandReturnType {
        // return version
        Ok(ResponseData::new())
    }

    fn run_get_serial_number(&self) -> CommandReturnType {
        // return serial number
        Ok(ResponseData::from_slice(b"1234").unwrap())
    }

    fn run_enable_timestamps(&self) -> CommandReturnType {
        // set timestamps on or off
        Ok(ResponseData::new())
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

enum CANBitrate {
    Bitrate10k,
    Bitrate20k,
    Bitrate50k,
    Bitrate100k,
    Bitrate125k,
    Bitrate250k,
    Bitrate500k,
    Bitrate800k,
    Bitrate1M,
}
