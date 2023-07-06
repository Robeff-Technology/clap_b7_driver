//
// Created by elaydin on 07.07.2023.
//

#include <clap_b7_driver/clap_b7_driver.hpp>


ClapB7Driver::ClapB7Driver() : Node("clap_b7_driver") {
    RCLCPP_INFO(this->get_logger(), "ClapB7Driver started");
}