//
// Created by elaydin on 09.07.2023.
//

#ifndef CLAP_CONFIG_H
#define CLAP_CONFIG_H

#include "rclcpp/rclcpp.hpp"
namespace clap_b7{

    class ConfigParams{
    private:
        std::string serial_port_;
        int baudrate_;
    public:
        ConfigParams();

        std::string get_serial_port();

        void load_parameters(const rclcpp::Node &node);

        int get_baudrate();
    };
}
#endif//CLAP_CONFIG_H
