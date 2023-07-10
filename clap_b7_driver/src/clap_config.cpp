//
// Created by elaydin on 09.07.2023.
//

#include <clap_b7_driver/clap_config.h>


namespace clap_b7{

    ConfigParams::ConfigParams() {
        serial_port_ = "/dev/ttyUSB0";
        baudrate_ = 460800;
    }

    void ConfigParams::load_parameters(const rclcpp::Node& node){
        node.get_parameter_or<std::string>("serial_config.port", serial_port_, "/dev/ttyUSB0");
        node.get_parameter_or<int>("serial_config.baudrate", baudrate_, 460800);
    }
    std::string ConfigParams::get_serial_port() {
        return serial_port_;
    }

    int ConfigParams::get_baudrate() {
        return baudrate_;
    }

} // namespace clap_b7