//
// Created by elaydin on 09.07.2023.
//

#ifndef CLAP_PUBSUB_H
#define CLAP_PUBSUB_H
#include <rclcpp/rclcpp.hpp>

#include <clap_b7_driver/msg/clap_gps_pos.hpp>
#include <clap_b7_driver/msg/clap_gps_vel.hpp>
#include <clap_b7_driver/msg/clap_heading.hpp>
#include <clap_b7_driver/msg/clap_imu.hpp>
#include <clap_b7_driver/msg/clap_ins.hpp>
#include <clap_b7_driver/msg/clap_ecef.hpp>
#include <clap_b7_driver/msg/clap_wheel_odom.hpp>

#include <clap_b7_driver/clap_config_params.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cstdint>


namespace clap_b7{
    class Publishers{
    private:
        uint32_t max_msg_size_ = 10;
        /*
         * std msgs Publishers
         */
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_raw_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped >::SharedPtr twist_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped >::SharedPtr twist_pub_ecef;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
        rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr gnss_ins_orientation_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_odom_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

        /*
         * Clap msgs Publishers
         */
        rclcpp::Publisher<clap_b7_driver::msg::ClapGpsPos>::SharedPtr gps_pos_pub_;
        rclcpp::Publisher<clap_b7_driver::msg::ClapGpsVel>::SharedPtr gps_vel_pub_;
        rclcpp::Publisher<clap_b7_driver::msg::ClapHeading>::SharedPtr heading_pub_;
        rclcpp::Publisher<clap_b7_driver::msg::ClapImu>::SharedPtr adis16470_imu_pub_;
        rclcpp::Publisher<clap_b7_driver::msg::ClapIns>::SharedPtr ins_pub_;
        rclcpp::Publisher<clap_b7_driver::msg::ClapECEF>::SharedPtr pub_ecef_;
        rclcpp::Publisher<clap_b7_driver::msg::ClapWheelOdom>::SharedPtr pub_wheel_odom_;
    public:

        void init_std_msgs_publisher(rclcpp::Node &ref_ros_node,clap_b7::ConfigParams params_);
        void init_custom_msgs_publisher(rclcpp::Node& ref_ros_node);

        void publish_imu(const sensor_msgs::msg::Imu& imu_msg);
        void publish_nav_sat_fix(const sensor_msgs::msg::NavSatFix& nav_sat_fix_msg);
        void publish_twist(const geometry_msgs::msg::TwistWithCovarianceStamped& twist_msg);

        void publish_gps_pos(const clap_b7_driver::msg::ClapGpsPos& gps_pos_msg);
        void publish_gps_vel(const clap_b7_driver::msg::ClapGpsVel& gps_vel_msg);
        void publish_heading(const clap_b7_driver::msg::ClapHeading& heading_msg);
        void publish_adis16470_imu(const clap_b7_driver::msg::ClapImu& adis16470_imu_msg);
        void publish_ins(const clap_b7_driver::msg::ClapIns& ins_msg);

        void publish_temperature(const sensor_msgs::msg::Temperature &temperature_msg);

        void publish_autoware_orientation(const autoware_sensing_msgs::msg::GnssInsOrientationStamped &autoware_orientation_msg);
        void publish_gnss_odom(const nav_msgs::msg::Odometry &gnss_odom_msg);

        void broadcast_transforms(const geometry_msgs::msg::TransformStamped &gnss_odom_tf_);

        void publish_ecef(const clap_b7_driver::msg::ClapECEF &ecef_msg);

        void publish_twist_ecef(const geometry_msgs::msg::TwistWithCovarianceStamped &twist_msg);

        void broadcast_static_transform(const geometry_msgs::msg::TransformStamped &gnss_odom_tf_);

        void publish_raw_navsatfix(const sensor_msgs::msg::NavSatFix &nav_sat_fix_msg);

        void publish_raw_imu(const sensor_msgs::msg::Imu &imu_msg);

        void publish_wheel_odom(const clap_b7_driver::msg::ClapWheelOdom &wheel_odom_msg);
    };

} // namespace clap_b7

#endif //CLAP_PUBSUB_H