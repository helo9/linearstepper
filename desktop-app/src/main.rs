#![windows_subsystem = "windows"]
use std::cell::RefCell;
use std::rc::Rc;
use log::info;

slint::include_modules!();

mod app_error;
mod arduino_port;
mod gui;

impl std::convert::From<slint::PlatformError> for app_error::AppError {
    fn from(_err: slint::PlatformError) -> Self {
        Self("Platform is not properly supported :(".to_string())
    }
}

fn main() -> Result<(), app_error::AppError> {

    env_logger::init();

    let ardu_conn: Option<arduino_port::ArduinoConnection> = None;

    let ui = AppWindow::new()?;

    let available_ports = arduino_port::collect_arduino_ports()?;


    ui.set_available_serial_ports(
        gui::create_ports_model(available_ports).into()
    );

    let port_option_rc = Rc::new(RefCell::new(ardu_conn));

    let port_option1 = Rc::clone(&port_option_rc);

    ui.on_send_forwards_command(move |val| {

        info!("send_forward_command is called with val {}", val);

        let msg = format!("g+{}\n", val);

        if let Some(ref mut p) = *port_option1.borrow_mut() {
            println!("Forward Command {} mm \n{:?}", val, msg.as_bytes());
            let _ = p.write(msg.as_bytes());
        }
    });

    let port_option2 = Rc::clone(&port_option_rc);

    ui.on_send_backwards_command(move |val| {

        info!("send_backwards_command is called with val {}", val);

        let msg = format!("g-{}\n", val);

        if let Some(ref mut p) = *port_option2.borrow_mut() {
            println!("Backwards Command {} mm \n{:?}", val, msg.as_bytes());
            let _ = p.write(msg.as_bytes());
        }
    });

    let port_option3 = Rc::clone(&port_option_rc);

    ui.on_connect(move |port_name| {

        info!("connect callback is called with port_name {}", &port_name);

        if let Ok(con) = arduino_port::open_connection(&port_name) {
            *port_option3.borrow_mut() = Some(con);
        }

        }
    );

    Ok(ui.run()?)
}
