mod util;

use crate::canbus::{CANBitrate, CANBus};
use crate::slcan::util::{concat, BoundedU8};
use bxcan::{ExtendedId, StandardId};
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
    CANError,
    BufferOverrun,
}

#[derive(Debug, Clone)]
pub struct QueueFullError;

#[derive(Debug, Clone)]
pub struct InvalidCommandError;

pub const COMMAND_TERMINATOR: u8 = b'\r';
pub const ERROR_CHAR: u8 = 7;

pub type QueueType = heapless::Deque<u8, 64>;
pub type CommandQueueType = heapless::Deque<Command, 8>;

trait HexOutput<const N: usize> {
    fn as_bytes(&self) -> [u8; N];

    fn as_hex(&self) -> [u8; N * 2] {
        let mut hex_str = [0u8; N * 2];
        hex::encode_to_slice(self.as_bytes(), &mut hex_str).unwrap();
        hex_str.make_ascii_uppercase();
        return hex_str;
    }

    fn push_to_queue(&self, queue: &mut QueueType) -> Result<[u8; N * 2], u8> {
        let hex_str = self.as_hex();
        for byte in hex_str {
            queue.push_front(byte)?;
        }
        Ok(hex_str)
    }
}

impl HexOutput<2> for StandardId {
    fn as_bytes(&self) -> [u8; 2] {
        self.as_raw().to_be_bytes()
    }
}

impl HexOutput<4> for ExtendedId {
    fn as_bytes(&self) -> [u8; 4] {
        self.as_raw().to_be_bytes()
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

    pub fn handle_incoming_can_frame(
        frame: &bxcan::Frame,
        tx_queue: &mut QueueType,
    ) -> Result<(), SLCANError> {
        SLCAN::can_frame_representation(frame, tx_queue)
            .map_err(|_e| -> SLCANError { SLCANError::Regular(ErrorKind::BufferOverrun) })
    }

    fn can_frame_representation(frame: &bxcan::Frame, queue: &mut QueueType) -> Result<(), u8> {
        match frame.id() {
            bxcan::Id::Standard(id) => {
                queue.push_front(b't')?;
                id.push_to_queue(queue)?;
            }
            bxcan::Id::Extended(id) => {
                queue.push_front(b'T')?;
                id.push_to_queue(queue)?;
            }
        }
        match frame.data() {
            Some(data) => {
                let len_char = char::from_digit(data.len() as u32, 10).unwrap();
                queue.push_front(len_char as u8)?;

                for i in 0..data.len() {
                    let mut hex_str = [0u8; 2];
                    hex::encode_to_slice([data[i]], &mut hex_str).unwrap();
                    hex_str.make_ascii_uppercase();
                    for byte in hex_str {
                        queue.push_front(byte)?;
                    }
                }

                // TODO send timestamps
            }
            None => {}
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
    pub fn run<I>(&self, slcan: &mut SLCAN, canbus: &mut CANBus<I>) -> CommandReturnType
    where
        I: bxcan::FilterOwner,
    {
        match self.variant {
            CommandVariant::Setup => self.run_setup(slcan, canbus),
            CommandVariant::SetupWithBTR => self.run_not_implemented(slcan),
            CommandVariant::OpenChannel => self.run_open_channel(slcan, canbus),
            CommandVariant::CloseChannel => self.run_close_channel(slcan, canbus),
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

    fn run_setup<I>(&self, slcan: &mut SLCAN, canbus: &mut CANBus<I>) -> CommandReturnType
    where
        I: bxcan::FilterOwner,
    {
        // set CAN bitrate
        let bitrate = match self.data.get(0) {
            Some(b'0') => CANBitrate::Bitrate10k,
            Some(b'1') => CANBitrate::Bitrate20k,
            Some(b'2') => CANBitrate::Bitrate50k,
            Some(b'3') => CANBitrate::Bitrate100k,
            Some(b'4') => CANBitrate::Bitrate125k,
            Some(b'5') => CANBitrate::Bitrate250k,
            Some(b'6') => CANBitrate::Bitrate500k,
            Some(b'7') => CANBitrate::Bitrate800k,
            Some(b'8') => CANBitrate::Bitrate1M,
            _ => return Err(SLCANError::Regular(ErrorKind::InvalidCommand)),
        };
        canbus
            .set_bitrate(bitrate)
            .map_err(|_e| SLCANError::Regular(ErrorKind::CANError))?;
        Ok(ResponseData::new())
    }

    fn run_open_channel<I>(&self, slcan: &mut SLCAN, canbus: &mut CANBus<I>) -> CommandReturnType
    where
        I: bxcan::FilterOwner,
    {
        // open the CAN channel
        canbus.enable();
        Ok(ResponseData::new())
    }

    fn run_close_channel<I>(&self, slcan: &mut SLCAN, canbus: &mut CANBus<I>) -> CommandReturnType
    where
        I: bxcan::FilterOwner,
    {
        // close the CAN channel
        canbus.disable();
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

// ****** end of command structs ******

struct StandardID {
    // 11-bit identifier
    id: u16,
}

struct ExtendedID {
    // 29-bit identifier
    id: u32,
}
