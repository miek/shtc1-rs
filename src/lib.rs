//! Driver for Sensirion SHTC1 digital humidity sensor

#![no_std]

extern crate byteorder;
extern crate embedded_hal;

use byteorder::{ByteOrder, BigEndian};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const CRC8_POLYNOMIAL: u8 = 0x31;
const I2C_ADDRESS: u8 = 0x70;

pub struct SHTC1<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D, E> SHTC1<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
	/// Creates a new driver
    pub fn new(i2c: I2C, delay: D) -> Self {
        SHTC1 { i2c, delay }
    }

	/// Send an I2C command
    fn command(&mut self, command: Command) -> Result<(), Error<E>> {
        self.i2c
            .write(I2C_ADDRESS, &command.value())
            .map_err(Error::I2c)
    }

    /// Take a temperature and humidity measurement
    pub fn measure(&mut self) -> Result<Measurement, Error<E>> {
        let raw = self.measure_raw()?;
        Ok(convert(&raw))
    }

    /// Take a temperature and humidity measurement
    pub fn measure_raw(&mut self) -> Result<MeasurementRaw, Error<E>> {
        self.command(Command::Measure(ClockStretch::Disabled, MeasurementOrder::TFirst))?;
        self.delay.delay_ms(15);
        let mut buf = [0; 6];
        self.i2c.read(I2C_ADDRESS, &mut buf)
                .map_err(Error::I2c)?;
        self.validate_crc(&buf[0..3])?;
        self.validate_crc(&buf[3..6])?;
        let temperature = BigEndian::read_u16(&buf[0..2]);
        let humidity = BigEndian::read_u16(&buf[3..5]);
        Ok(MeasurementRaw{ temperature, humidity })
    }

    /// Read the ID register
    pub fn read_id(&mut self) -> Result<u16, Error<E>> {
        self.command(Command::ReadID)?;
        let mut id_bytes = [0; 3];
        self.i2c
            .read(I2C_ADDRESS, &mut id_bytes)
            .map_err(Error::I2c)?;
        self.validate_crc(&id_bytes[0..3])?;
        Ok(BigEndian::read_u16(&id_bytes[0..2]))
    }

    pub fn release(self) -> I2C {
        self.i2c
    }

    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.command(Command::SoftReset)?;
        Ok(())
    }

    fn validate_crc(&self, data: &[u8]) -> Result<(), Error<E>> {
        match crc8(data) {
            0x00 => Ok(()),
            _ => Err(Error::Crc),
        }
    }
}

/// Convert MeasurementRaw to Measurement
pub fn convert(m: &MeasurementRaw) -> Measurement {
    Measurement{
        temperature: convert_temperature(m.temperature),
        humidity: convert_humidity(m.humidity),
    }
}

fn convert_temperature(raw: u16) -> i32 {
    -4500 + (17500 * raw as i32) / 65535
}

fn convert_humidity(raw: u16) -> i32 {
    (10000 * raw as i32) / 65535
}

fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xff;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) > 0 {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Wrong CRC
    Crc,
    /// I2C bus error
    I2c(E),
}

enum Command {
    Measure(ClockStretch, MeasurementOrder),
    SoftReset,
    ReadID,
}

enum ClockStretch {
    Enabled,
    Disabled,
}

#[derive(Copy, Clone)]
pub enum MeasurementOrder {
    TFirst,
    HFirst,
}

#[derive(Debug)]
pub struct Measurement {
    pub temperature: i32,
    pub humidity: i32,
}

#[derive(Debug)]
pub struct MeasurementRaw {
    pub temperature: u16,
    pub humidity: u16,
}

impl Command {
    fn value(&self) -> [u8; 2] {
        use ClockStretch::Enabled as CSEnabled;
        use ClockStretch::Disabled as CSDisabled;
        use MeasurementOrder::*;
        match *self {
            // 5.2 Measurement Commands
            // Table 9
            Command::Measure(CSEnabled,  TFirst) => [0x7Cu8, 0xA2u8],
            Command::Measure(CSEnabled,  HFirst) => [0x5Cu8, 0x24u8],
            Command::Measure(CSDisabled, TFirst) => [0x78u8, 0x66u8],
            Command::Measure(CSDisabled, HFirst) => [0x58u8, 0xE0u8],

            // 5.6 Soft Reset
            // Table 10
            Command::SoftReset => [0x80, 0x5D],

            // 5.7 Read-out of ID register
            // Table 11
            Command::ReadID  => [0xEF, 0xC8],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn crc() {
        assert_eq!(crc8(&[0x00u8]), 0xAC);
        assert_eq!(crc8(&[0xBEu8, 0xEFu8]), 0x92);
    }
}
