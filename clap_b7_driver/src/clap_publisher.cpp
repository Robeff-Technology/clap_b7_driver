

#include <clap_b7_driver/clap_publisher.h>


namespace clap_b7 {

    void Publishers::init_std_msgs_publisher(rclcpp::Node &ref_ros_node,
        clap_b7::ConfigParams params_) {
        /*
         * std msgs publishers
         */
        imu_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::Imu>(params_.get_imu_topic(), max_msg_size_);
        nav_sat_fix_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::NavSatFix>(params_.get_nav_sat_fix_topic(), max_msg_size_);
        twist_pub_ = ref_ros_node.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(params_.get_twist_topic(), max_msg_size_);
        temperature_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::Temperature>("clap/ros/temperature", max_msg_size_);
    }

    void Publishers::init_custom_msgs_publisher(rclcpp::Node &ref_ros_node) {
        /*
         * clap msgs publishers
         */
        gps_pos_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapGpsPos>("clap/clap_gnss_pos", max_msg_size_);
        gps_vel_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapGpsVel>("clap/clap_gnss_vel", max_msg_size_);
        adis16470_imu_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapImu>("clap/clap_adis16470", max_msg_size_);
        ins_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapIns>("clap/clap_ins", max_msg_size_);
    }

    void Publishers::publish_ins(const clap_b7_driver::msg::ClapIns &ins_msg) {
        if(ins_pub_){
            ins_pub_->publish(ins_msg);
        }

    }

    void Publishers::publish_temperature(const sensor_msgs::msg::Temperature& temperature_msg){
        if(temperature_pub_){
            temperature_pub_->publish(temperature_msg);
        }
    }
    void Publishers::publish_imu(const sensor_msgs::msg::Imu& imu_msg){
        if(imu_pub_){
            imu_pub_->publish(imu_msg);
        }
    }
    void Publishers::publish_nav_sat_fix(const sensor_msgs::msg::NavSatFix& nav_sat_fix_msg){
        if(nav_sat_fix_pub_){
            nav_sat_fix_pub_->publish(nav_sat_fix_msg);
        }
    }
    void Publishers::publish_twist(const geometry_msgs::msg::TwistWithCovarianceStamped& twist_msg){
        if(twist_pub_){
            twist_pub_->publish(twist_msg);
        }
    }
    void Publishers::publish_gps_pos(const clap_b7_driver::msg::ClapGpsPos& gps_pos_msg){
        if(gps_pos_pub_){
            gps_pos_pub_->publish(gps_pos_msg);
        }
    }
    void Publishers::publish_gps_vel(const clap_b7_driver::msg::ClapGpsVel& gps_vel_msg){
        if(gps_vel_pub_){
            gps_vel_pub_->publish(gps_vel_msg);
        }
    }
    void Publishers::publish_heading(const clap_b7_driver::msg::ClapHeading& heading_msg){
        if(heading_pub_) {
            heading_pub_->publish(heading_msg);
        }
    }
    void Publishers::publish_adis16470_imu(const clap_b7_driver::msg::ClapImu& adis16470_imu_msg){
        if(adis16470_imu_pub_){
            adis16470_imu_pub_->publish(adis16470_imu_msg);
        }
    }
} // namespace clap_b7