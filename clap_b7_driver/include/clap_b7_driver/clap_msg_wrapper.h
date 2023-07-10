//
// Created by elaydin on 07.07.2023.
//

#ifndef CLAP_MSG_WRAPPER_H
#define CLAP_MSG_WRAPPER_H

#include <cstdint>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <clap_b7_driver/clap_structs.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <clap_b7_driver/msg/clap_heading.hpp>
#include <clap_b7_driver/msg/clap_imu.hpp>
#include <clap_b7_driver/msg/clap_gps_pos.hpp>
#include <clap_b7_driver/msg/clap_gps_vel.hpp>
#include <clap_b7_driver/msg/clap_ins.hpp>



namespace clap_b7{
    class ClapMsgWrapper {
    private:
        bool use_ros_time_{true};
        int64_t clap_timestamp{};
        static double raw_gyro_to_deg_s(int32_t raw_gyro);
        static double raw_acc_to_m_s2(int32_t raw_acc) ;
    public:
        ClapMsgWrapper() = default;
        ~ClapMsgWrapper();
        sensor_msgs::msg::Imu create_sensor_imu_msg(const clap_b7::RawImu& raw_imu, const clap_b7::InsPvax& ins, std::string frame_id) const;
        static double degree2radian(double degree) ;
        void set_system_time(int64_t timestamp);
        void set_use_ros_time(bool use_ros_time);
        static bool is_ins_active(const clap_b7::InsPvax& ins);
        sensor_msgs::msg::NavSatFix create_nav_sat_fix_msg(const InsPvax& ins, std::string frame_id) const;
        sensor_msgs::msg::NavSatFix create_nav_sat_fix_msg(const BestGnssPos& ins, std::string frame_id) const;

        geometry_msgs::msg::TwistWithCovarianceStamped
        create_twist_msg(const BestGnssVel& gnss_vel, float heading, int32_t z_gyro_raw, std::string frame_id) const;

        std_msgs::msg::Header create_header(std::string frame_id) const;

        clap_b7_driver::msg::ClapGpsPos create_gps_pos_msg(const BestGnssPos &gnss_pos, std::string frame_id) const;

        clap_b7_driver::msg::ClapGpsVel create_gps_vel_msg(const BestGnssVel &gnss_vel, std::string frame_id) const;

        clap_b7_driver::msg::ClapHeading create_gps_heading_msg(const UniHeading &uniheading, std::string frame_id) const;

        clap_b7_driver::msg::ClapImu create_imu_msg(const RawImu &raw_imu, std::string frame_id) const;

        clap_b7_driver::msg::ClapIns create_ins_msg(const InsPvax &ins, std::string frame_id) const;
    };

} // namespace clap_b7

#endif//CLAP_MSG_WRAPPER_H