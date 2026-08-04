#![no_std]

//! # INA238 driver for embedded-hal
//! A Rust driver for the INA238 current/voltage/power monitor IC.
//! This driver is designed to work with the `embedded-hal` traits, making it compatible with a wide range of microcontrollers and platforms.
//! More about the embedded-hal: https://github.com/rust-embedded/embedded-hal
//!
//! Todo : Feature complete implementation of the INA238 driver, threshold configuration and handling alerts.

use embedded_hal::i2c::I2c;

/// Custom error type for the crate, representing possible errors that can occur during I2C communication or device operation.
#[derive(Debug)]
pub enum Error<E> {
    /// I2C communication error, wrapping the underlying error type `E`.
    Communication(E),
}

/// Possible device addresses based on INA238 Add0/A0 and Add1/A1 pin configurations.
#[derive(Debug, Clone, Copy)]
pub enum Address {
    AddrA1gndA0gnd = 0x40, // A1=GND, A0=GND (default)
    AddrA1gndA0vs = 0x41,  // A1=GND, A0=VS
    AddrA1gndA0sda = 0x42, // A1=GND, A0=SDA
    AddrA1gndA0scl = 0x43, // A1=GND, A0=SCL
    AddrA1vsA0gnd = 0x44,  // A1=VS,  A0=GND
    AddrA1vsA0vs = 0x45,   // A1=VS,  A0=VS
    AddrA1vsA0sda = 0x46,  // A1=VS,  A0=SDA
    AddrA1vsA0scl = 0x47,  // A1=VS,  A0=SCL
    AddrA1sdaA0gnd = 0x48, // A1=SDA, A0=GND
    AddrA1sdaA0vs = 0x49,  // A1=SDA, A0=VS
    AddrA1sdaA0sda = 0x4A, // A1=SDA, A0=SDA
    AddrA1sdaA0scl = 0x4B, // A1=SDA, A0=SCL
    AddrA1sclA0gnd = 0x4C, // A1=SCL, A0=GND
    AddrA1sclA0vs = 0x4D,  // A1=SCL, A0=VS
    AddrA1sclA0sda = 0x4E, // A1=SCL, A0=SDA
    AddrA1sclA0scl = 0x4F, // A1=SCL, A0=SCL
}

impl Address {
    /// Returns the I2C address as a u8 value.
    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

#[allow(dead_code)]
pub enum AdcRange {
    Range163mV,
    Range40mV,
}

pub enum Mode {
    Shutdown = 0x0,
    TriggeredBus = 0x1,
    TriggeredShunt = 0x2,
    TriggeredBusShunt = 0x3,
    TriggeredTemp = 0x4,
    TriggeredTempBus = 0x5,
    TriggeredTempShunt = 0x6,
    TriggeredAll = 0x7,
    ContinuousBus = 0x9,
    ContinuousShunt = 0xA,
    ContinuousBusShunt = 0xB,
    ContinuousTemp = 0xC,
    ContinuousTempBus = 0xD,
    ContinuousTempShunt = 0xE,
    ContinuousAll = 0xF,
}
pub enum Time {
    Ct50,
    Ct84,
    Ct150,
    Ct280,
    Ct540,
    Ct1052,
    Ct2074,
    Ct4120,
}

#[allow(dead_code)]
struct Register;

#[allow(dead_code)]
impl Register {
    const CONFIG: u8 = 0x00;
    const ADC_CONFIG: u8 = 0x01;
    const SHUNT_CAL: u8 = 0x02;
    const VSHUNT: u8 = 0x04;
    const VBUS: u8 = 0x05;
    const DIETEMP: u8 = 0x06;
    const CURRENT: u8 = 0x07;
    const POWER: u8 = 0x08;
    const DIAG_ALRT: u8 = 0x0B;
    const SOVL: u8 = 0x0C;
    const SUVL: u8 = 0x0D;
    const BOVL: u8 = 0x0E;
    const BUVL: u8 = 0x0F;
    const TEMP_LIMIT: u8 = 0x10;
    const PWR_LIMIT: u8 = 0x11;
    const MANUFACTURER_ID: u8 = 0x3E;
    const DEVICE_ID: u8 = 0x3F;
}

/// INA238 sensor driver
/// Driver struct
pub struct INA238<I2C> {
    i2c: I2C,
    address: Address,
    current_lsb: f32,      // Current LSB value in amperes
    shunt_resistance: f32, // Shunt resistance in ohms
}

/// Methods for the INA238 driver
impl<I2C> INA238<I2C>
where
    I2C: I2c,
{
    /// Creates a new INA238 driver instance
    pub fn new(i2c: I2C, address: Address) -> Self {
        assert!(
            0x40 <= address.as_u8() && address.as_u8() <= 0x4F,
            "Invalid I2C address for INA238 (0x40 to 0x4F are valid)"
        );
        Self {
            i2c,
            address,
            current_lsb: 0.0,
            shunt_resistance: 0.0,
        }
    }

    /// Default initialization of the INA238 sensor
    pub fn with_default_address(i2c: I2C) -> Self {
        Self::new(i2c, Address::AddrA1gndA0gnd)
    }

    /// Returns the I2C address of the INA238 sensor
    pub fn address(&self) -> Address {
        self.address
    }

    /// Returns the manufacturer ID of the sensor
    pub fn manufacture_id(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_u16(Register::MANUFACTURER_ID)
    }

    /// Returns the device ID of the sensor
    pub fn device_id(&mut self) -> Result<(u16, u8), Error<I2C::Error>> {
        let device_id = self.read_u16(Register::DEVICE_ID)?;
        let dei_id = device_id >> 4;
        let rev_id: u8 = (device_id & 0x0F) as u8;
        Ok((dei_id, rev_id))
    }

    /// Soft reset to INA238 sensor, resets all registers to their default values.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_u16(Register::CONFIG, 1 << 15)
    }

    /// Configures the INA238 sensor
    pub fn set_config(
        &mut self,
        conv_delay: u16,
        range: AdcRange,
    ) -> Result<(), Error<I2C::Error>> {
        let mut value = (conv_delay / 2) << 6;
        value = match range {
            AdcRange::Range40mV => value | (1 << 4),
            AdcRange::Range163mV => value & !(1 << 4),
        };
        self.write_u16(Register::CONFIG, value)
    }

    /// ADC configuration, conversion times can be configured using this method.
    pub fn set_adc_config(
        &mut self,
        mode: Mode,
        busvolt_ct: Time,
        shuntvolt_ct: Time,
        temperature_ct: Time,
        adc_avgcount: u8,
    ) -> Result<(), Error<I2C::Error>> {
        let value = (mode as u16) << 12
            | (busvolt_ct as u16) << 9
            | (shuntvolt_ct as u16) << 6
            | (temperature_ct as u16) << 3
            | (adc_avgcount as u16);
        self.write_u16(Register::ADC_CONFIG, value)
    }

    /// Sets the shunt calibration value, which is used to calculate the current and power measurements.
    /// Note: The calibration value should be calculated based on the shunt resistor value and the desired current range. RSHUNT < (VSENSE_MAX/I_MAX)
    pub fn set_shunt_calibrate(
        &mut self,
        max_current_a: f32,
        shunt_resistance: f32,
    ) -> Result<(), Error<I2C::Error>> {
        self.current_lsb = max_current_a / 32768.0;
        self.shunt_resistance = shunt_resistance;
        self.write_shunt_calibrate(self.current_lsb, self.shunt_resistance)
    }

    /// Shunt voltage measurement, returns the shunt voltage in volts.
    pub fn shunt_voltage(&mut self) -> Result<f32, Error<I2C::Error>> {
        let raw_value: i16 = self.read_i16(Register::VSHUNT)?;

        let conv_factor = self.adc_conv_factor()?;
        Ok((raw_value as f32) * conv_factor as f32 * 1.25e-6)
    }

    /// Bus voltage measurement, returns the bus voltage in volts.
    pub fn bus_voltage(&mut self) -> Result<f32, Error<I2C::Error>> {
        let raw_value = self.read_i16(Register::VBUS)?;
        Ok((raw_value as f32) * 3.125e-3)
    }

    /// Temperature measurement, returns the die temperature in degrees Celsius.
    pub fn die_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let raw_value = self.read_i16(Register::DIETEMP)? >> 4;
        Ok((raw_value as f32) * 125e-3)
    }

    /// Current measurement, returns the current in amperes.
    pub fn current(&mut self) -> Result<f32, Error<I2C::Error>> {
        let raw_value = self.read_i16(Register::CURRENT)?;
        Ok(raw_value as f32 * self.current_lsb)
    }
    /// Power measurement, returns the power in watts.
    pub fn power(&mut self) -> Result<f32, Error<I2C::Error>> {
        let raw_value = self.read_i24(Register::POWER)?;
        Ok(raw_value as f32 * 0.2 * self.current_lsb)
    }

    
    ///Threshold registers write & read
    /// Set Shunt overvoltage threshold in volts
    pub fn set_shunt_overvoltage_th(&mut self, voltage_v: f32) -> Result<(), Error<I2C::Error>> {
        let conv_factor = 1.25e-6 * (self.adc_conv_factor()?) as f32;
        let raw_value = (voltage_v / conv_factor) as i16;
        self.write_u16(Register::SOVL, raw_value as u16)
    }

    /// Set Shunt undervoltage threshold in volts
    pub fn set_shunt_undervoltage_th(&mut self, voltage_v: f32) -> Result<(), Error<I2C::Error>> {
        let conv_factor = 1.25e-6 * (self.adc_conv_factor()?) as f32;
        let raw_value = (voltage_v / conv_factor) as i16;
        self.write_u16(Register::SUVL, raw_value as u16)
    }

    /// Set Bus overvoltage threshold in volts
    pub fn set_bus_overvoltage_th(&mut self, voltage_v: f32) -> Result<(), Error<I2C::Error>> {
        let raw_value = (voltage_v / 3.125e-3) as u16;
        self.write_u16(Register::BOVL, raw_value)
    }

    /// Set Bus undervoltage threshold in volts
    pub fn set_bus_undervoltage_th(&mut self, voltage_v: f32) -> Result<(), Error<I2C::Error>> {
        let raw_value = (voltage_v / 3.125e-3) as u16;
        self.write_u16(Register::BUVL, raw_value)
    }

    /// Sets temperature over limit in Celcius
    pub fn set_temperature_limit(&mut self, temp_c: f32) -> Result<(), Error<I2C::Error>> {
        let raw_value = (temp_c / 125e-3) as i16;
        self.write_u16(Register::TEMP_LIMIT, (raw_value as u16) << 4)
    }

    /// Sets power over limit threshold in Watts
    pub fn set_power_limit(&mut self, power_w: f32) -> Result<(), Error<I2C::Error>> {
        let power_lsb = self.current_lsb * 0.2;
        let raw_value = (power_w / (256.0 * power_lsb)) as u16;
        self.write_u16(Register::TEMP_LIMIT, raw_value)
    }

    ///---- Private functions ----
    ///
    /// Conversion factor for the ADC based on the configuration register.
    /// Returns 1 for 40mV range (1.25uV per bit) and 4 for 163mV range (5uV per bit).
    fn adc_conv_factor(&mut self) -> Result<u16, Error<I2C::Error>> {
        let adc_range_bit = ((self.read_u16(Register::CONFIG)? >> 1) & 1) != 0;
        let conv_factor = if adc_range_bit {
            1 // 1.25uV per bit for 40mV range
        } else {
            4 // 5uV per bit for 163mV range -> 1.25uV x 4 = 5uV
        };
        Ok(conv_factor)
    }

    /// Calculates and writes the shunt calibration value to the sensor.
    fn write_shunt_calibrate(
        &mut self,
        current_lsb: f32,
        shunt_resistance_ohms: f32,
    ) -> Result<(), Error<I2C::Error>> {
        let conv_factor = self.adc_conv_factor()?;
        let shunt_cal: f32 = 819.2e6 * current_lsb * shunt_resistance_ohms * conv_factor as f32;
        self.write_u16(Register::SHUNT_CAL, shunt_cal as u16)
    }

    ///-----I2C helper functions -----
    //
    fn read_u16(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        match self.i2c.write_read(self.address.as_u8(), &[reg], &mut buf) {
            Ok(()) => Ok(u16::from_be_bytes(buf)),
            Err(e) => Err(Error::Communication(e)),
        }
    }

    fn read_i16(&mut self, reg: u8) -> Result<i16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        match self.i2c.write_read(self.address.as_u8(), &[reg], &mut buf) {
            Ok(()) => Ok(i16::from_be_bytes(buf)),
            Err(e) => Err(Error::Communication(e)),
        }
    }

    fn read_i24(&mut self, reg: u8) -> Result<i32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        match self.i2c.write_read(self.address.as_u8(), &[reg], &mut buf) {
            Ok(()) => Ok({
                let bytes = [0, buf[0], buf[1], buf[2]];
                i32::from_be(i32::from_ne_bytes(bytes))
            }),
            Err(e) => Err(Error::Communication(e)),
        }
    }

    fn write_u16(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let bytes = value.to_be_bytes();
        match self
            .i2c
            .write(self.address.as_u8(), &[reg, bytes[0], bytes[1]])
        {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::Communication(e)),
        }
    }
}
