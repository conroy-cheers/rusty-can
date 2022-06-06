pub struct BoundedU8<const MIN: u8, const MAX: u8>(u8);

impl<const MIN: u8, const MAX: u8> BoundedU8<MIN, MAX> {
    pub const fn new(value: u8) -> Result<Self, ()> {
        if value >= MIN && value <= MAX {
            Ok(Self(value))
        } else {
            Err(())
        }
    }
}
