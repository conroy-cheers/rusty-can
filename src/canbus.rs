use bxcan::{self, filter::Mask32, Frame, TransmitStatus};
use can_bit_timings::can_timings_bxcan;
use nb::block;

#[derive(Debug)]
pub enum CANError {
    Regular(ErrorKind),
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ErrorKind {
    InvalidTiming,
    BufferOverrun,
}

pub enum CANBitrate {
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

pub struct CANBus<I>
where
    I: bxcan::FilterOwner,
{
    can_instance: bxcan::Can<I>,
}

impl<I> CANBus<I>
where
    I: bxcan::FilterOwner,
{
    pub fn new(can: I) -> Self {
        let mut bxcan = bxcan::Can::builder(can).leave_disabled();
        let mut filters = bxcan.modify_filters();
        filters.enable_bank(0, Mask32::accept_all());
        drop(filters);

        CANBus {
            can_instance: bxcan,
        }
    }

    pub fn transmit(&mut self, frame: &Frame) -> TransmitStatus {
        block!(self.can_instance.transmit(frame)).unwrap()
    }

    pub fn receive(&mut self) -> Result<Frame, CANError> {
        block!(self.can_instance.receive())
            .map_err(|()| -> CANError { CANError::Regular(ErrorKind::BufferOverrun) })
    }

    pub fn set_bitrate(&mut self, bitrate: CANBitrate) -> Result<(), CANError> {
        let timings = CANBus::<I>::get_bit_timings(bitrate)?;

        let config = self.can_instance.modify_config();
        config.set_bit_timing(timings).leave_disabled();

        Ok(())
    }

    pub fn enable(&mut self) {
        self.can_instance.modify_config().enable();
    }

    pub fn disable(&mut self) {
        self.can_instance.modify_config().leave_disabled();
    }

    fn get_bit_timings(bitrate: CANBitrate) -> Result<u32, CANError> {
        match bitrate {
            CANBitrate::Bitrate10k => Ok(can_timings_bxcan!(8.mhz(), 10.khz())),
            CANBitrate::Bitrate20k => Ok(can_timings_bxcan!(8.mhz(), 20.khz())),
            CANBitrate::Bitrate50k => Ok(can_timings_bxcan!(8.mhz(), 50.khz())),
            CANBitrate::Bitrate100k => Ok(can_timings_bxcan!(8.mhz(), 100.khz())),
            CANBitrate::Bitrate125k => Ok(can_timings_bxcan!(8.mhz(), 125.khz())),
            CANBitrate::Bitrate250k => Ok(can_timings_bxcan!(8.mhz(), 250.khz())),
            CANBitrate::Bitrate500k => Ok(can_timings_bxcan!(8.mhz(), 500.khz())),
            CANBitrate::Bitrate800k => Ok(can_timings_bxcan!(8.mhz(), 800.khz())),
            CANBitrate::Bitrate1M => Err(CANError::Regular(ErrorKind::InvalidTiming)),
        }
    }
}
