//
// Created by elaydin on 07.07.2023.
//
#include <clap_b7_driver/clap_msg_wrapper.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

constexpr double accel_scale_factor = 0.000000186;
constexpr double hz_to_second = 100;
constexpr double gyro_scale_factor = 0.000001006;
namespace clap_b7{
    ClapMsgWrapper::~ClapMsgWrapper() {

    }
    void ClapMsgWrapper::set_system_time(int64_t timestamp) {
        clap_timestamp = timestamp;
    }

    bool ClapMsgWrapper::is_ins_active(clap_b7::InsPvax& ins){
        return ins.ins_status > 1;
    }

    double ClapMsgWrapper:: raw_acc_to_m_s2(int32_t raw_acc) {
        return static_cast<double>(raw_acc) * accel_scale_factor * hz_to_second;
    }
    double ClapMsgWrapper::raw_gyro_to_deg_s(int32_t raw_gyro) {
        return static_cast<double>(raw_gyro) * gyro_scale_factor * hz_to_second;
    }

    double ClapMsgWrapper::degree2radian(double degree) {
        return degree * M_PI / 180.0;
    }

    sensor_msgs::msg::NavSatFix ClapMsgWrapper::create_nav_sat_fix_msg(clap_b7::InsPvax& ins, std::string frame_id) const {
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
        if(use_ros_time_) {
            nav_sat_fix_msg.header.stamp = rclcpp::Clock().now();
        }
        else {
            nav_sat_fix_msg.header.stamp = rclcpp::Time(clap_timestamp);
        }
        nav_sat_fix_msg.header.frame_id = frame_id;

        if(ins.pos_type == 56) {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        else if(ins.pos_type == 54){
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        }
        else {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }

        nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        nav_sat_fix_msg.latitude = ins.latitude;
        nav_sat_fix_msg.longitude = ins.longitude;
        nav_sat_fix_msg.altitude = ins.height; //TODO: ~yee sbg altitude = height + undulation ask Leo team
        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        nav_sat_fix_msg.position_covariance[0] = ins.std_dev_latitude * ins.std_dev_latitude;
        nav_sat_fix_msg.position_covariance[4] = ins.std_dev_longitude * ins.std_dev_longitude;
        nav_sat_fix_msg.position_covariance[8] = ins.std_dev_height * ins.std_dev_height;

        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return nav_sat_fix_msg;
    }

    sensor_msgs::msg::NavSatFix ClapMsgWrapper::create_nav_sat_fix_msg(clap_b7::BestGnssPos &gps_pos, std::string frame_id) const{
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
        if(use_ros_time_) {
            nav_sat_fix_msg.header.stamp = rclcpp::Clock().now();
        }
        else {
            nav_sat_fix_msg.header.stamp = rclcpp::Time(clap_timestamp);
        }
        nav_sat_fix_msg.header.frame_id = frame_id;

        if(gps_pos.pos_type == 56) {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        else if(gps_pos.pos_type == 54){
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        }
        else {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }

        nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        nav_sat_fix_msg.latitude = gps_pos.latitude;
        nav_sat_fix_msg.longitude = gps_pos.longitude;
        nav_sat_fix_msg.altitude = gps_pos.height; //TODO: ~yee sbg altitude = height + undulation ask Leo team
        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        nav_sat_fix_msg.position_covariance[0] = gps_pos.std_dev_latitude * gps_pos.std_dev_latitude;
        nav_sat_fix_msg.position_covariance[4] = gps_pos.std_dev_longitude * gps_pos.std_dev_longitude;
        nav_sat_fix_msg.position_covariance[8] = gps_pos.std_dev_height * gps_pos.std_dev_height;

        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return nav_sat_fix_msg;
    }

    sensor_msgs::msg::Imu ClapMsgWrapper::create_sensor_imu_msg(clap_b7::RawImu& raw_imu, clap_b7::InsPvax& ins, std::string frame_id) const {
        sensor_msgs::msg::Imu imu_msg;
        if(use_ros_time_) {
            imu_msg.header.stamp = rclcpp::Clock().now();
        }
        else {
            imu_msg.header.stamp = rclcpp::Time(clap_timestamp);
        }
        imu_msg.header.frame_id = frame_id;
        tf2::Quaternion q;
        /*
         * in clap b7 roll-> y axis pitch-> x axis azimuth->left handed rotation around z-axis
         * in ros imu msg roll-> x axis pitch-> y axis azimuth->right handed rotation around z-axis
         */
        q.setRPY(degree2radian(ins.pitch), degree2radian(ins.roll), degree2radian(-ins.azimuth));
        imu_msg.orientation.w = q.getW();
        imu_msg.orientation.x = q.getX();
        imu_msg.orientation.y = q.getY();
        imu_msg.orientation.z = q.getZ();

        imu_msg.angular_velocity.x = degree2radian(raw_gyro_to_deg_s(raw_imu.x_gyro_output));
        imu_msg.angular_velocity.y = degree2radian(raw_gyro_to_deg_s(raw_imu.y_gyro_output));
        imu_msg.angular_velocity.z = degree2radian(raw_gyro_to_deg_s(raw_imu.z_gyro_output));

        imu_msg.linear_acceleration.x = degree2radian(raw_acc_to_m_s2(raw_imu.x_accel_output));
        imu_msg.linear_acceleration.y = degree2radian(raw_acc_to_m_s2(raw_imu.y_accel_output));
        imu_msg.linear_acceleration.z = degree2radian(raw_acc_to_m_s2(raw_imu.z_accel_output));

        imu_msg.orientation_covariance[0] = ins.std_dev_pitch * ins.std_dev_pitch;
        imu_msg.orientation_covariance[4] = ins.std_dev_roll * ins.std_dev_roll;
        imu_msg.orientation_covariance[8] = ins.std_dev_pitch * ins.std_dev_pitch;

        /*
         * angular velocity and linear acceleration covariance is not provided by clap b7
         */
        for(size_t i = 0; i < 9; i++){
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
        }
        return imu_msg;
    }

    void ClapMsgWrapper::set_use_ros_time(bool use_ros_time) {
        use_ros_time_ = use_ros_time;
    }


}