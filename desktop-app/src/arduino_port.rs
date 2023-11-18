//! An module for serial ports handling

use serialport::{available_ports, SerialPort};

use crate::app_error::{AppError};

const BAUDRATE: u32 = 19_200;

impl std::convert::From<serialport::Error> for AppError {
    fn from(err: serialport::Error) -> Self {
        Self(format!("serialport: {}", err.description))
    }
}

pub fn collect_arduino_ports() -> Result<Vec<String>,AppError> {
    let ports = available_ports()?;

    // TODO: Implement filtering, to get rid of none arduino ports

    let port_names: Vec<String> = ports.into_iter()
        .map(|e| e.port_name.into())
        .collect();

    Ok(port_names)
}

pub type ArduinoConnection = Box<dyn SerialPort>;

pub fn open_connection(port_name: &str) -> Result<ArduinoConnection, AppError> {

    let p = serialport::new(port_name, BAUDRATE)
        .timeout(std::time::Duration::from_millis(10))
        .open()?;

    Ok(p)
}
