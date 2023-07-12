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
        node.get_parameter_or<std::string>("topic_config.imu_topic",imu_topic_,"clap/ros/imu");
        node.get_parameter_or<std::string>("topic_config.nav_sat_fix_topic",nav_sat_fix_topic_,"clap/ros/nav_sat_fix");
        node.get_parameter_or<std::string>("topic_config.twist_topic",twist_topic_,"clap/ros/twist");
        node.get_parameter_or<std::string>("topic_config.temperature_topic",temperature_topic_,"clap/ros/temperature");
        node.get_parameter_or<std::string>("topic_config.autoware_orientation_topic",autoware_orientation_topic_,"clap/autoware_orientation");
        node.get_parameter_or<std::string>("frame_config.gnss_frame",gnss_frame_,"GNSS_INS/gnss_ins_link");
        node.get_parameter_or<double>("true_heading_config.true_heading_offset",true_heading_offset_,0.0);
        node.get_parameter_or<bool>("odometry_config.use_odemetry",use_odemetry_,false);
        node.get_parameter_or<std::string>("odometry_config.odemetry_topic",odemetry_topic_,"clap/ros/odometry");
        node.get_parameter_or<std::string>("odometry_config.odometry_frame",odometry_frame_,"odom");
        node.get_parameter_or<double>("origin_config.lat_origin",lat_origin_,0.0);
        node.get_parameter_or<double>("origin_config.long_origin",long_origin_,0.0);
        node.get_parameter_or<double>("origin_config.alt_origin",alt_origin_,0.0);
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

    std::string ConfigParams::get_imu_topic() {
        return imu_topic_;
    }

    std::string ConfigParams::get_nav_sat_fix_topic() {
        return nav_sat_fix_topic_;
    }

    std::string ConfigParams::get_twist_topic() {
        return twist_topic_;
    }

    std::string ConfigParams::get_gnss_frame() {
        return gnss_frame_;
    }

    std::string ConfigParams::get_temperature_topic() {
        return temperature_topic_;
    }

    std::string ConfigParams::get_autoware_orientation_topic(){
        return autoware_orientation_topic_;
    }

    double ConfigParams::get_true_heading_offset() {
        return true_heading_offset_;
    }

    bool ConfigParams::get_use_odometry(){
        return use_odemetry_;
    }

    std::string ConfigParams::get_odometry_topic(){
        return odemetry_topic_;
    }

    std::string ConfigParams::get_odometry_frame(){
        return odometry_frame_;
    }

    double ConfigParams::get_lat_origin(){
        return lat_origin_;
    }

    double ConfigParams::get_long_origin(){
        return long_origin_;
    }

    double ConfigParams::get_alt_origin(){
        return alt_origin_;
    }


} // namespace clap_b7