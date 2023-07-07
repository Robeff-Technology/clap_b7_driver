//
// Created by elaydin on 07.07.2023.
//

#ifndef CLAP_MSG_WRAPPER_H
#define CLAP_MSG_WRAPPER_H

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <clap_b7_driver/clap_structs.h>

#include <cstdint>

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
        sensor_msgs::msg::Imu create_sensor_imu_msg(clap_b7::RawImu& raw_imu, clap_b7::InsPvax& ins, std::string frame_id) const;
        static double degree2radian(double degree) ;
        void set_system_time(int64_t timestamp);
        void set_use_ros_time(bool use_ros_time);
        bool is_ins_active(clap_b7::InsPvax& ins);
        sensor_msgs::msg::NavSatFix create_nav_sat_fix_msg(InsPvax &ins, std::string frame_id) const;
        sensor_msgs::msg::NavSatFix create_nav_sat_fix_msg(BestGnssPos &ins, std::string frame_id) const;
    };

} // namespace clap_b7

#endif//CLAP_MSG_WRAPPER_H
