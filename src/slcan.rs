mod util;

use crate::canbus::{CANBitrate, CANBus};
use crate::slcan::util::concat;
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

fn err_invalid_command(_e: hex::FromHexError) -> SLCANError {
    SLCANError::Regular(ErrorKind::InvalidCommand)
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

pub const COMMAND_TERMINATOR: u8 = b'\r';
pub const ERROR_CHAR: u8 = 7;

pub type QueueType = heapless::Deque<u8, 128>;

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
            queue.push_back(byte)?;
        }
        Ok(hex_str)
    }
}

impl HexOutput<2> for StandardId {
    fn as_bytes(&self) -> [u8; 2] {
        self.as_raw().to_be_bytes()
    }

    fn push_to_queue(&self, queue: &mut QueueType) -> Result<[u8; 4], u8> {
        let hex_str = self.as_hex();
        for byte in &hex_str[1..] {
            queue.push_back(*byte)?;
        }
        Ok(hex_str)
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
    pub bitrate: Option<CANBitrate>,
    timestamps_enabled: bool,
    status: StatusFlags,
    version: VersionInfo,
    serial_number: [u8; 4],
}

impl SLCAN {
    pub fn new() -> Self {
        SLCAN {
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
        match &result {
            Err(SLCANError::Regular(kind)) => match kind {
                ErrorKind::QueueFull => self.status.receive_queue_full = true,
                ErrorKind::InvalidCommand => self.status.error_passive = true,
                ErrorKind::NotImplemented => self.status.error_passive = true,
                ErrorKind::CANError => self.status.bus_error = true,
                ErrorKind::BufferOverrun => self.status.transmit_queue_full = true,
            },
            Ok(_c) => {}
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
        rx_queue.push_back(incoming_byte).map_err(err_queue_full)?;
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
                    tx_queue.push_back(*b)?;
                }
                tx_queue.push_back(crate::slcan::COMMAND_TERMINATOR)?;
            }
            Err(_e) => {
                tx_queue.push_back(crate::slcan::ERROR_CHAR)?;
            }
        }
        Ok(())
    }

    pub fn handle_incoming_can_frame(
        frame: &bxcan::Frame,
        tx_queue: &mut QueueType,
    ) -> Result<(), SLCANError> {
        let repr = SLCAN::can_frame_representation(frame, true);

        let available = tx_queue.capacity() - tx_queue.len();
        // need 1 extra space for terminator
        if repr.len() >= available {
            return Err(SLCANError::Regular(ErrorKind::BufferOverrun));
        }
        
        for byte in repr {
            tx_queue.push_back(byte).unwrap();
        }
        tx_queue.push_back(COMMAND_TERMINATOR).unwrap();

        Ok(())
    }

    fn can_frame_representation(
        frame: &bxcan::Frame,
        include_start_byte: bool,
    ) -> heapless::Vec<u8, 24> {
        let mut rep = heapless::Vec::<u8, 24>::new();

        if include_start_byte {
            let start_byte = match frame.id() {
                bxcan::Id::Standard(_id) => b"t",
                bxcan::Id::Extended(_id) => b"T",
            };
            rep.extend_from_slice(start_byte).unwrap();
        }

        match frame.id() {
            bxcan::Id::Standard(id) => {
                rep.extend_from_slice(&id.as_hex()[1..]).unwrap();
            }
            bxcan::Id::Extended(id) => {
                rep.extend_from_slice(&id.as_hex()).unwrap();
            }
        }

        match frame.data() {
            Some(data) => {
                let data_len: u8 = char::from_digit(data.len() as u32, 10).unwrap() as u8;
                rep.extend_from_slice(&[data_len]).unwrap();

                let mut data_str = heapless::Vec::<u8, 16>::new();
                for i in 0..data.len() {
                    let mut hex_str = [0u8; 2];
                    hex::encode_to_slice([data[i]], &mut hex_str).unwrap();
                    hex_str.make_ascii_uppercase();

                    data_str.extend_from_slice(&hex_str).unwrap();
                }
                rep.extend(data_str);

                // TODO send timestamps
            }
            None => {
                let data_len = b"0";
                rep.extend_from_slice(data_len).unwrap();
            }
        }

        return rep;
    }
}

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
    data: heapless::Vec<u8, 32>,
}

type RequestData = heapless::Vec<u8, 32>;
pub type ResponseData = heapless::Vec<u8, 32>;
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
        slcan.bitrate = Some(bitrate);
        Ok(ResponseData::new())
    }

    fn run_open_channel<I>(&self, _slcan: &mut SLCAN, canbus: &mut CANBus<I>) -> CommandReturnType
    where
        I: bxcan::FilterOwner,
    {
        // open the CAN channel
        canbus.enable();
        Ok(ResponseData::new())
    }

    fn run_close_channel<I>(&self, _slcan: &mut SLCAN, canbus: &mut CANBus<I>) -> CommandReturnType
    where
        I: bxcan::FilterOwner,
    {
        // close the CAN channel
        canbus.disable();
        Ok(ResponseData::new())
    }

    fn run_transmit_frame(&self, _slcan: &mut SLCAN) -> CommandReturnType {
        // transmit a frame
        // frame must have minimum 4 bytes
        if self.data.len() < 4 {
            return Err(SLCANError::Regular(ErrorKind::InvalidCommand));
        }

        let mut id = [0u8; 2];
        hex::decode_to_slice(&self.data[1..4], &mut id).map_err(err_invalid_command)?;
        let id = bxcan::StandardId::new(u16::from_be_bytes(id))
            .ok_or(SLCANError::Regular(ErrorKind::InvalidCommand))?;

        let mut data_len = [0u8; 1];
        hex::decode_to_slice(&self.data[4..5], &mut data_len).map_err(err_invalid_command)?;
        let data_len: usize = u8::from_be_bytes(data_len).into();

        // ensure frame size is correct
        let last_idx: usize = (4 + 2 * data_len).into();
        if data_len > 8 || self.data.len() != last_idx {
            return Err(SLCANError::Regular(ErrorKind::InvalidCommand));
        }

        let mut data = [0u8; 8];
        hex::decode_to_slice(&self.data[5..last_idx], &mut data).map_err(err_invalid_command)?;

        let frame = bxcan::Frame::new_data(id, bxcan::Data::new(&data[..data_len]).unwrap());

        let frame_bytes = SLCAN::can_frame_representation(&frame, true);
        Ok(ResponseData::from_slice(frame_bytes.as_slice()).unwrap())
    }

    fn run_transmit_extended_frame(&self, _slcan: &mut SLCAN) -> CommandReturnType {
        // transmit an extended frame
        // frame must have minimum 9 bytes
        if self.data.len() < 9 {
            return Err(SLCANError::Regular(ErrorKind::InvalidCommand));
        }
        let mut id = [0u8; 4];
        hex::decode_to_slice(&self.data[1..9], &mut id).map_err(err_invalid_command)?;
        let id = bxcan::ExtendedId::new(u32::from_be_bytes(id))
            .ok_or(SLCANError::Regular(ErrorKind::InvalidCommand))?;

        let mut data_len = [0u8; 1];
        hex::decode_to_slice(&self.data[9..10], &mut data_len).map_err(err_invalid_command)?;
        let data_len: usize = u8::from_be_bytes(data_len).into();

        // ensure frame size is correct
        let last_idx: usize = (9 + 2 * data_len).into();
        if data_len > 8 || self.data.len() != last_idx {
            return Err(SLCANError::Regular(ErrorKind::InvalidCommand));
        }

        let mut data = [0u8; 8];
        hex::decode_to_slice(&self.data[10..last_idx], &mut data).map_err(err_invalid_command)?;

        let frame = bxcan::Frame::new_data(id, bxcan::Data::new(&data[..data_len]).unwrap());

        let frame_bytes = SLCAN::can_frame_representation(&frame, true);
        Ok(ResponseData::from_slice(frame_bytes.as_slice()).unwrap())
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
