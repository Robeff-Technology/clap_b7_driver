//
// Created by elaydin on 07.07.2023.
//

#ifndef CLAP_MSG_WRAPPER_H
#define CLAP_MSG_WRAPPER_H

#include <cstdint>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <clap_b7_driver/clap_structs.h>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <clap_b7_driver/msg/clap_heading.hpp>
#include <clap_b7_driver/msg/clap_imu.hpp>
#include <clap_b7_driver/msg/clap_gps_pos.hpp>
#include <clap_b7_driver/msg/clap_gps_vel.hpp>
#include <clap_b7_driver/msg/clap_ins.hpp>
#include <clap_b7_driver/msg/clap_ecef.hpp>
#include <clap_b7_driver/msg/clap_wheel_odom.hpp>

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <Eigen/Dense>


namespace clap_b7{
    class ClapMsgWrapper {
    private:
        bool use_ros_time_{true};
        bool ins_initialized_{false};
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
        static double calc_imu_temperature(const clap_b7::RawImu& raw_imu);
        sensor_msgs::msg::NavSatFix create_nav_sat_fix_msg(const InsPvax& ins, std::string frame_id, int alt_mode) const;
        sensor_msgs::msg::NavSatFix create_nav_sat_fix_msg(const BestGnssPos& gps_pos, std::string frame_id, int alt_mode) const;

        geometry_msgs::msg::TwistWithCovarianceStamped
        create_twist_msg(const BestGnssVel& gnss_vel, float heading, const RawImu& imu, std::string frame_id) const;
        std_msgs::msg::Header create_header(std::string frame_id) const;

        clap_b7_driver::msg::ClapGpsPos create_gps_pos_msg(const BestGnssPos &gnss_pos, std::string frame_id) const;

        clap_b7_driver::msg::ClapGpsVel create_gps_vel_msg(const BestGnssVel &gnss_vel, std::string frame_id) const;

        clap_b7_driver::msg::ClapHeading create_gps_heading_msg(const UniHeading &uniheading, std::string frame_id) const;

        clap_b7_driver::msg::ClapImu create_imu_msg(const RawImu &raw_imu, std::string frame_id) const;

        clap_b7_driver::msg::ClapIns create_ins_msg(const InsPvax &ins, std::string frame_id) const;

        sensor_msgs::msg::Temperature create_temperature_msg(const RawImu &raw_imu, std::string frame_id) const;

        autoware_sensing_msgs::msg::GnssInsOrientationStamped create_autoware_orientation_msg(const InsPvax &ins, const UniHeading& heading, std::string frame_id) const;
        static double add_heading_offset(double heading, double offset);

        geometry_msgs::msg::TransformStamped
        create_transform(const geometry_msgs::msg::Pose &ref_pose, std::string frame_id, std::string child_frame_id) const;

        static Eigen::Matrix<double, 3, 3> convert_stddev_llh_to_enu(const Eigen::Matrix<double, 3, 3> &covarianceLLH,
                                                              const Eigen::Matrix<double, 3, 3> &rotationMatrix);

        nav_msgs::msg::Odometry
        create_odom_msg(const InsPvax &ins, const RawImu &imu, double x, double y, double z,
                        std::string frame_id, std::string child_frame_id) const;

        geometry_msgs::msg::TwistWithCovarianceStamped
        create_twist_msg(const ECEF &ecef, const RawImu& imu, std::string frame_id) const;

        clap_b7_driver::msg::ClapECEF create_ecef_msg(const ECEF &ecef) const;

        sensor_msgs::msg::Imu create_raw_imu_msg(const RawImu &imu, std::string frame_id) const;

        static double raw_gyro_to_deg(int32_t raw_gyro);

        static double scale_angle(double angle);

        clap_b7_driver::msg::ClapWheelOdom create_wheel_odom_msg(const TimeDWheelData &wheel_odom) const;

        bool is_ins_initialized(const InsPvax &ins);

        static bool is_delay_high(int64_t clap_timestamp);
    };

} // namespace clap_b7

#endif//CLAP_MSG_WRAPPER_H
