//
// Created by elaydin on 25.07.2023.
//

#include <rclcpp/rclcpp.hpp>
#include "clap_b7_driver/config_clap_b7.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<clap_b7::ConfigClap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}