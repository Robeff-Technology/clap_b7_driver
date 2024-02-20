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
        bool pub_custom_msgs_;
        bool sub_rtcm_msgs_;
        bool use_local_origin_;
        std::string imu_topic_;
        std::string nav_sat_fix_topic_;
        std::string twist_topic_;
        std::string temperature_topic_;
        std::string autoware_orientation_topic_;
        std::string gnss_frame_;
        std::string imu_frame_;
        double true_heading_offset_;
        int altitude_mode_;

        bool use_odometry_;
        std::string odometry_topic_;
        std::string odometry_frame_;
        double lat_origin_;
        double long_origin_;
        double alt_origin_;
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

        std::string get_imu_topic();

        std::string get_nav_sat_fix_topic();

        std::string get_twist_topic();

        std::string get_gnss_frame();

        std::string get_imu_frame();

        std::string get_temperature_topic();

        std::string get_autoware_orientation_topic();

        double get_true_heading_offset();

        bool get_use_odometry();

        std::string get_odometry_topic();

        std::string get_odometry_frame();

        double get_lat_origin();

        double get_long_origin();

        double get_alt_origin();

        bool get_use_local_origin();

        int get_altitude_mode();
    };
}
#endif//CLAP_CONFIG_H
