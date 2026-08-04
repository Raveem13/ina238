#[cfg(test)]

mod tests {
    /// Import driver structures, functions from the parent module
    use ina238::*;

    /// std library for testing
    extern crate std;

    /// Embedded-hal-mock library for testing I2C communication
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    const DEFAULT_ADDRESS: Address = Address::AddrA1gndA0gnd; // Default I2C address for INA238

    #[test]
    fn default_address() {
        let mut i2c = I2cMock::new(&[]);
        INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        i2c.done();
    }

    #[test]
    fn test_manufacture_id() {
        let mut i2c = I2cMock::new(&[read_txn(0x3E, &[0x54, 0x49])]);
        let mut ina = INA238::new(i2c.clone(), DEFAULT_ADDRESS);
        let id = ina.manufacture_id().unwrap();
        println!("Manufacture ID: 0x{:04X}", id);
        i2c.done();
    }

    // ---- Helper functions ----
    fn read_txn(reg: u8, bytes: &[u8]) -> I2cTransaction {
        I2cTransaction::write_read(DEFAULT_ADDRESS as u8, vec![reg], bytes.to_vec())
    }
}
