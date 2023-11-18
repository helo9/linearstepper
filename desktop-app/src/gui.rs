slint::include_modules!();

pub fn create_ports_model(port_list: Vec<String>) -> std::rc::Rc<slint::VecModel<slint::SharedString>> {
    let slint_port_list: Vec<slint::SharedString> = port_list
        .into_iter()
        .map(|e| slint::SharedString::from(e))
        .collect();

    std::rc::Rc::new(slint::VecModel::from(slint_port_list))
}
