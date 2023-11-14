use std::time::Duration;
use std::cell::RefCell;
use std::rc::Rc;
use serialport::{available_ports,SerialPortInfo};
slint::include_modules!();

const BAUDRATE: u32 = 19_200;

#[derive(Debug)]
pub struct AppError(String);

impl std::fmt::Display for AppError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for AppError {}

impl std::convert::From<serialport::Error> for AppError {
    fn from(err: serialport::Error) -> Self {
        Self(format!("serialport: {}", err.description))
    }
}

impl std::convert::From<slint::PlatformError> for AppError {
    fn from(err: slint::PlatformError) -> Self {
        Self("Platform is not properly supported :(".to_string())
    }
}

fn main() -> Result<(), AppError> {

    use slint::Model;

    let port = {
        let p = serialport::new("/dev/ttyUSB0", BAUDRATE)
            .timeout(Duration::from_millis(10))
            .open();

        if p.is_err() {
            println!("Couldn't open serial port ({:?})", p.as_ref().err())
        }

        p.ok()
    };

    println!("Port is good? ({:?})", port);

    let ui = AppWindow::new()?;

    let arduino_ports: Vec<slint::SharedString> = collect_arduino_ports()?
                            .into_iter()
                            .map(|e| slint::SharedString::from(e))
                            .collect();


    let port_model = std::rc::Rc::new(
        slint::VecModel::from(arduino_ports)
    );

    ui.set_available_serial_ports(
        port_model.into()
    );

    let port_option_rc = Rc::new(RefCell::new(port));

    let port_option1 = Rc::clone(&port_option_rc);

    ui.on_send_forwards_command(move |val| {
        let msg = format!("g+{}\n", val);

        if let Some(ref mut p) = *port_option1.borrow_mut() {
            println!("Forward Command {} mm \n{:?}", val, msg.as_bytes());
            let _ = p.write(msg.as_bytes());
        }
    });

    let port_option2 = Rc::clone(&port_option_rc);

    ui.on_send_backwards_command(move |val| {
        let msg = format!("g-{}\n", val);

        if let Some(ref mut p) = *port_option2.borrow_mut() {
            println!("Backwards Command {} mm \n{:?}", val, msg.as_bytes());
            let _ = p.write(msg.as_bytes());
        }
    });

    Ok(ui.run()?)
}

fn collect_arduino_ports() -> Result<Vec<String>,AppError> {
    let ports = available_ports()?;

    let port_names: Vec<String> = ports.into_iter()
        .map(|e| e.port_name.into())
        .collect();

    Ok(port_names)
}
