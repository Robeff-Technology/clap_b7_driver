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
        std::string rtcm_topic_;
        int baudrate_;
        bool use_ros_time_;
        bool pub_std_msgs_;
        bool pub_custom_msgs;
        bool sub_rtcm_msgs;
    public:
        ConfigParams();

        std::string get_serial_port();

        void load_parameters(const rclcpp::Node &node);

        int get_baudrate();

        bool get_use_ros_time();

        bool get_pub_std_msgs();

        bool get_pub_custom_msgs();

        bool get_sub_ntrip_msgs();

        std::string get_rtcm_topic();
    };
}
#endif//CLAP_CONFIG_H
