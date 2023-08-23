//
// Created by elaydin on 09.07.2023.
//

#include <clap_b7_driver/clap_config_params.h>


namespace clap_b7{

    ConfigParams::ConfigParams() {
    }

    void ConfigParams::load_parameters(const rclcpp::Node& node){
        node.get_parameter_or<std::string>("serial_config.port", serial_port_, "/dev/ttyUSB0");
        node.get_parameter_or<int>("serial_config.baudrate", baudrate_, 460800);
        node.get_parameter_or<bool>("time_config.use_ros_time", use_ros_time_, true);
        node.get_parameter_or<bool>("topic_config.pub_standard_msgs", pub_std_msgs_, true);
        node.get_parameter_or<bool>("topic_config.pub_custom_msgs", pub_custom_msgs_, true);
        node.get_parameter_or<bool>("topic_config.sub_rtcm_msgs", sub_rtcm_msgs_, true);
        node.get_parameter_or<std::string>("topic_config.rtcm_topic_name", rtcm_topic_, "/sensing/gnss/ntrip/rtcm");
        node.get_parameter_or<std::string>("topic_config.imu_topic",imu_topic_,"clap/ros/imu");
        node.get_parameter_or<std::string>("topic_config.nav_sat_fix_topic",nav_sat_fix_topic_,"clap/ros/nav_sat_fix");
        node.get_parameter_or<std::string>("topic_config.twist_topic",twist_topic_,"clap/ros/twist");
        node.get_parameter_or<std::string>("topic_config.temperature_topic",temperature_topic_,"clap/ros/temperature");
        node.get_parameter_or<std::string>("topic_config.autoware_orientation_topic",autoware_orientation_topic_,"clap/autoware_orientation");
        node.get_parameter_or<std::string>("frame_config.gnss_frame",gnss_frame_,"GNSS_INS/gnss_ins_link");
        node.get_parameter_or<double>("true_heading_config.true_heading_offset",true_heading_offset_,0.0);
        node.get_parameter_or<bool>("odometry_config.use_odometry",use_odometry_,false);
        node.get_parameter_or<std::string>("odometry_config.odometry_topic",odometry_topic_,"clap/ros/odometry");
        node.get_parameter_or<std::string>("odometry_config.odometry_frame",odometry_frame_,"odom");
        node.get_parameter_or<double>("origin_config.latitude",lat_origin_,0.0);
        node.get_parameter_or<double>("origin_config.longitude",long_origin_,0.0);
        node.get_parameter_or<double>("origin_config.altitude",alt_origin_,0.0);
        node.get_parameter_or<bool>("origin_config.use_local_origin",use_local_origin_, false);

        RCLCPP_INFO(node.get_logger(), "Configurations are loaded.");
        RCLCPP_INFO(node.get_logger(), "----------------------------------------------------------------");
        RCLCPP_INFO(node.get_logger(), "serial_port: %s", serial_port_.c_str());
        RCLCPP_INFO(node.get_logger(), "baudrate: %d", baudrate_);
        RCLCPP_INFO(node.get_logger(), "use_ros_time: %d", use_ros_time_);
        RCLCPP_INFO(node.get_logger(), "pub_std_msgs: %d", pub_std_msgs_);
        RCLCPP_INFO(node.get_logger(), "pub_custom_msgs: %d", pub_custom_msgs_);
        RCLCPP_INFO(node.get_logger(), "sub_rtcm_msgs: %d", sub_rtcm_msgs_);
        RCLCPP_INFO(node.get_logger(), "rtcm_topic: %s", rtcm_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "imu_topic: %s", imu_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "nav_sat_fix_topic: %s", nav_sat_fix_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "twist_topic: %s", twist_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "temperature_topic: %s", temperature_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "autoware_orientation_topic: %s", autoware_orientation_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "gnss_frame: %s", gnss_frame_.c_str());
        RCLCPP_INFO(node.get_logger(), "true_heading_offset: %f", true_heading_offset_);
        RCLCPP_INFO(node.get_logger(), "use_odometry: %d", use_odometry_);
        RCLCPP_INFO(node.get_logger(), "odometry_topic: %s", odometry_topic_.c_str());
        RCLCPP_INFO(node.get_logger(), "odometry_frame: %s", odometry_frame_.c_str());
        RCLCPP_INFO(node.get_logger(), "use_local_origin: %d", use_local_origin_);
        RCLCPP_INFO(node.get_logger(), "lat_origin: %f", lat_origin_);
        RCLCPP_INFO(node.get_logger(), "long_origin: %f", long_origin_);
        RCLCPP_INFO(node.get_logger(), "alt_origin: %f", alt_origin_);
        RCLCPP_INFO(node.get_logger(), "----------------------------------------------------------------");
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
        return pub_custom_msgs_;
    }

    bool ConfigParams::get_sub_ntrip_msgs() {
        return sub_rtcm_msgs_;
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
        return use_odometry_;
    }

    std::string ConfigParams::get_odometry_topic(){
        return odometry_topic_;
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

    bool ConfigParams::get_use_local_origin(){
        return use_local_origin_;
    }


} // namespace clap_b7