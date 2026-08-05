/// Tests for the INA238 driver using a mock I2C implementation.
/// embedded-hal provides a mock I2C implementation that can be used for testing purposes.
#[cfg(test)]
mod tests {
    /// Import top-level structures, functions from the parent module
    use ina238::*;

    /// std library for testing
    extern crate std;

    use embedded_hal::i2c::{Error as I2cError, ErrorKind, Operation};

    //I2C stub
    #[derive(Debug)]
    pub struct I2cStub {
        pub response_data: [u8; 2],
        pub call_count: usize,
    }

    /// I2C bus stub implementation
    impl I2cStub {
        // create new I2C bus
        pub fn new() -> Self {
            Self {
                response_data: [0x00, 0x00],
                call_count: 0,
            }
        }

        pub fn set_manufactureid(&mut self, id: u16) {
            self.response_data = id.to_be_bytes();
        }
    }

    // Declare a dummy error type
    #[derive(Debug, Clone)]
    pub struct DummyError;

    //Implement the I2c trait for the I2cStub
    impl I2cError for DummyError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    // Associated type: use our DummyError as the error type for the I2c trait
    impl embedded_hal::i2c::ErrorType for I2cStub {
        type Error = DummyError;
    }

    //Stub implementation of the I2c trait read and write methods
    impl embedded_hal::i2c::I2c for I2cStub {
        // read method returns always ok
        fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        // write method returns always ok
        fn write(&mut self, _address: u8, _write: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        // Fill out read buffer with the response data and return ok
        fn write_read(
            &mut self,
            _address: u8,
            _write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            // Fill read buffer with a canned response
            read.copy_from_slice(&self.response_data);
            self.call_count += 1;
            Ok(())
        }

        // Transaction method returns always ok
        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    // Test 1: Create new driver, & device address is valid, should not panic
    #[test]
    fn test_new_driver_valid_address() {
        let i2c = I2cStub::new();
        let driver = INA238::new(i2c, Address::AddrA1gndA0gnd);
        assert_eq!(driver.address().as_u8(), 0x40);
    }

    // Test 2: Read the manufacture ID using stub
    #[test]
    fn test_manufactureid() {
        // create a new I2C stub
        let mut i2c = I2cStub::new();

        // set the manufacture ID to a known value
        i2c.set_manufactureid(0x5449);

        let mut driver = INA238::new(i2c, Address::AddrA1gndA0gnd);
        // Read the manufacture id
        let id = driver.manufacture_id().unwrap();

        assert_eq!(id, 0x5449);
    }
}
