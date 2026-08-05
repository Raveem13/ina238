#[cfg(test)]

mod tests {

    /// Import driver structures, functions from the parent module
    use ina238::{AdcRange::Range40mV, Count::AvgN1024, Mode::ContinuousAll, Time::Ct1052, *};

    /// std library for testing
    extern crate std;

    /// Embedded-hal-mock library for testing I2C communication
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    const DEFAULT_ADDRESS: Address = Address::AddrA1gndA0gnd; // Default I2C address for INA238

    //Test for default address
    #[test]
    fn default_address() {
        let mut i2c = I2cMock::new(&[]);
        INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        i2c.done();
    }

    //Test for address max bound
    #[test]
    fn max_address() {
        let mut i2c = I2cMock::new(&[]);
        INA238::new(i2c.clone(), Address::AddrA1vsA0vs);
        i2c.done();
    }

    // Reset test
    #[test]
    fn reset_test() {
        let mut i2c = I2cMock::new(&[write_txn(0x00, 1 << 15), read_txn(0x05, &[0x00, 0x00])]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.reset().unwrap();
        //Check busvoltage after reset
        assert_eq!(ina.bus_voltage().unwrap(), 0.0);
        i2c.done();
    }

    // Test to get manufacture ID
    #[test]
    fn test_manufacture_id() {
        let mut i2c = I2cMock::new(&[read_txn(0x3E, &[0x54, 0x49])]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        let id = ina.manufacture_id().unwrap();
        assert_eq!(id, 0x5449);
        i2c.done();
    }

    // Test to get device ID
    #[test]
    fn test_device_id() {
        let mut i2c = I2cMock::new(&[read_txn(0x3F, &[0x23, 0x81])]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        let did = ina.device_id().unwrap();
        assert_eq!(did, (0x238, 0x1));
        i2c.done();
    }

    // Calibration test
    #[test]
    fn test_calibration() {
        let shunt_cal = exp_shunt_cal(20.0, 0.1, false);
        let mut i2c = I2cMock::new(&[write_txn(0x02, shunt_cal)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_shunt_calibrate(20.0, 0.1).unwrap();
        i2c.done();
    }

    // Configuration test
    #[test]
    fn test_configuration() {
        let confg_val = exp_confg(10, true);
        let mut i2c = I2cMock::new(&[write_txn(0x00, confg_val)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_config(10, Range40mV).unwrap();
        i2c.done();
    }

    // ADC Configuration test
    #[test]
    fn test_adc_configuration() {
        let exp_val = exp_adc_cfg(ContinuousAll, Ct1052, Ct1052, Ct1052, AvgN1024);
        let mut i2c = I2cMock::new(&[write_txn(0x01, exp_val)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_adc_config(ContinuousAll, Ct1052, Ct1052, Ct1052, AvgN1024)
            .unwrap();
        i2c.done();
    }

    // Shunt voltage test
    #[test]
    fn test_shunt_volt() {
        let mut i2c = I2cMock::new(&[read_txn(0x04, &0x0020_u16.to_be_bytes())]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.shunt_voltage().unwrap();
        i2c.done();
    }

    // Bus voltage test
    #[test]
    fn test_bus_volt() {
        let mut i2c = I2cMock::new(&[read_txn(0x05, &0x0010_u16.to_be_bytes())]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.bus_voltage().unwrap();
        i2c.done();
    }

    // Internal die temperature test
    #[test]
    fn test_die_temp() {
        let mut i2c = I2cMock::new(&[read_txn(0x06, &0x0040_u16.to_be_bytes())]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.die_temperature().unwrap();
        i2c.done();
    }

    // Current result test
    #[test]
    fn test_current() {
        let mut i2c = I2cMock::new(&[read_txn(0x07, &0x0001_u16.to_be_bytes())]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.current().unwrap();
        i2c.done();
    }

    // Power result test
    #[test]
    fn test_power() {
        let mut i2c = I2cMock::new(&[read_txn(0x08, &u24_be_bytes(0x0020))]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.power().unwrap();
        i2c.done();
    }

    // Test shunt overvoltage threshold
    #[test]
    fn test_shunt_overvolt() {
        // 0x7FFF (32727) * 1.25uv = 0.04 V
        let exp_val = exp_shunt_volt(0.04, true) as u16;
        let mut i2c = I2cMock::new(&[write_txn(0x0C, exp_val)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_shunt_overvoltage_th(0.04).unwrap();
        i2c.done();
    }

    // Test shunt undervoltage threshold
    #[test]
    fn test_shunt_undervolt() {
        // 0x8000 (-32728) * 1.25uv = -0.04 V
        let exp_val = exp_shunt_volt(-0.04, true) as u16;
        let mut i2c = I2cMock::new(&[write_txn(0x0D, exp_val)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_shunt_undervoltage_th(-0.04).unwrap();
        i2c.done();
    }

    // Test Bus overvoltage threshold
    #[test]
    fn test_bus_overvolt() {
        // 0x7FFF (32727) * 3.125mv = 102.2 V
        let exp_val = (102.0 / 3.125e-3) as u16;
        let mut i2c = I2cMock::new(&[write_txn(0x0E, exp_val)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_bus_overvoltage_th(102.0).unwrap();
        i2c.done();
    }

    // Test bus undervoltage threshold
    #[test]
    fn test_bus_undervolt() {
        // 0x1 * 3.125mv = 0.003 V
        let exp_val = (0.003 / 3.125e-3) as u16;
        let mut i2c = I2cMock::new(&[write_txn(0x0F, exp_val)]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        ina.set_bus_undervoltage_th(0.003).unwrap();
        i2c.done();
    }

    // ---- Helper functions ----
    fn read_txn(reg: u8, bytes: &[u8]) -> I2cTransaction {
        I2cTransaction::write_read(DEFAULT_ADDRESS as u8, vec![reg], bytes.to_vec())
    }

    fn write_txn(reg: u8, value: u16) -> I2cTransaction {
        let bytes = value.to_be_bytes();
        I2cTransaction::write(DEFAULT_ADDRESS as u8, vec![reg, bytes[0], bytes[1]])
    }

    fn u24_be_bytes(value: u32) -> [u8; 3] {
        [(value >> 16) as u8, (value >> 8) as u8, value as u8]
    }

    //Expected shunt constant (SHUNT_CAL) calculation
    fn exp_shunt_cal(max_current: f32, shunt_resistance: f32, adc_range: bool) -> u16 {
        let current_lsb = max_current / (1 << 15) as f32;
        let mut shunt_val = 819.2e6 * current_lsb * shunt_resistance;
        if adc_range {
            shunt_val *= 4.0;
        }
        shunt_val as u16
    }

    // Expected config register value calculation
    fn exp_confg(conv_delay_ms: u16, adc_range: bool) -> u16 {
        let mut cfg_val = (conv_delay_ms >> 1) << 6;
        if adc_range {
            cfg_val |= 1 << 4;
        }
        cfg_val as u16
    }

    // Expected ADC config register value calculation
    fn exp_adc_cfg(
        mode: Mode,
        busvolt_ct: Time,
        shuntvolt_ct: Time,
        tempe_ct: Time,
        adc_avgcount: Count,
    ) -> u16 {
        (mode as u16) << 12
            | (busvolt_ct as u16) << 9
            | (shuntvolt_ct as u16) << 6
            | (tempe_ct as u16) << 3
            | adc_avgcount as u16
    }

    // Shunt voltage calculations
    fn exp_shunt_volt(volt: f32, adc_range: bool) -> i16 {
        let mut shunt_volt = volt / 5e-6;
        if adc_range {
            shunt_volt *= 4.0;
        }
        println!("The value is: {:#x}", shunt_volt as u16);
        shunt_volt as i16
    }
}
