//
// Created by elaydin on 09.07.2023.
//

#include <clap_b7_driver/clap_config.h>


namespace clap_b7{

    ConfigParams::ConfigParams() {
    }

    void ConfigParams::load_parameters(const rclcpp::Node& node){
        node.get_parameter_or<std::string>("serial_config.port", serial_port_, "/dev/ttyUSB0");
        node.get_parameter_or<int>("serial_config.baudrate", baudrate_, 460800);
        node.get_parameter_or<bool>("time_config.use_ros_time", use_ros_time_, true);
        node.get_parameter_or<bool>("topic_config.pub_std_msgs", pub_std_msgs_, true);
        node.get_parameter_or<bool>("topic_config.pub_custom_msgs", pub_custom_msgs, true);
        node.get_parameter_or<bool>("topic_config.sub_rtcm_msgs", sub_rtcm_msgs, true);
        node.get_parameter_or<std::string>("topic_config.rtcm_topic_name", rtcm_topic_, "/sensing/gnss/ntrip/rtcm");
    }
    std::string ConfigParams::get_serial_port() {
        return serial_port_;
    }

    int ConfigParams::get_baudrate() {
        return baudrate_;
    }

    bool ConfigParams::get_use_ros_time() {
        return use_ros_time_;
    }

    bool ConfigParams::get_pub_std_msgs() {
        return pub_std_msgs_;
    }

    bool ConfigParams::get_pub_custom_msgs() {
        return pub_custom_msgs;
    }

    bool ConfigParams::get_sub_ntrip_msgs() {
        return sub_rtcm_msgs;
    }

    std::string ConfigParams::get_rtcm_topic() {
        return rtcm_topic_;
    }

} // namespace clap_b7