

#include <clap_b7_driver/clap_publisher.h>



namespace clap_b7 {

    void Publishers::init_std_msgs_publisher(rclcpp::Node &ref_ros_node,
        clap_b7::ConfigParams params_) {
        /*
         * std msgs publishers
         */
        temperature_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::Temperature>(params_.get_temperature_topic(), max_msg_size_);
        twist_pub_ = ref_ros_node.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(params_.get_twist_topic(), max_msg_size_);
        twist_pub_ecef = ref_ros_node.create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("raw/ecef_twist", max_msg_size_);
        nav_sat_fix_raw_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::NavSatFix>("raw/nav_sat_fix", max_msg_size_);
        nav_sat_fix_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::NavSatFix>(params_.get_nav_sat_fix_topic(), max_msg_size_);
        imu_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::Imu>(params_.get_imu_topic(), max_msg_size_);
        raw_imu_pub_ = ref_ros_node.create_publisher<sensor_msgs::msg::Imu>("/raw/imu", max_msg_size_);
        gnss_ins_orientation_pub_ = ref_ros_node.create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(params_.get_autoware_orientation_topic(), max_msg_size_);

        if(params_.get_use_odometry()){
            gnss_odom_pub_ = ref_ros_node.create_publisher<nav_msgs::msg::Odometry>(params_.get_odometry_topic(), max_msg_size_);
            tf_broadcaster_odom_ = std::make_shared<tf2_ros::TransformBroadcaster>(ref_ros_node);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(ref_ros_node);
        }
    }

    void Publishers::init_custom_msgs_publisher(rclcpp::Node &ref_ros_node) {
        /*
         * clap msgs publishers
         */
        gps_pos_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapGpsPos>("clap/clap_gnss_pos", max_msg_size_);
        gps_vel_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapGpsVel>("clap/clap_gnss_vel", max_msg_size_);
        adis16470_imu_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapImu>("clap/clap_adis16470", max_msg_size_);
        ins_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapIns>("clap/clap_ins", max_msg_size_);
        pub_ecef_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapECEF>("clap/ecef_pos", max_msg_size_);
        pub_wheel_odom_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapWheelOdom>("clap/wheel_odom", max_msg_size_);
        heading_pub_ = ref_ros_node.create_publisher<clap_b7_driver::msg::ClapHeading>("clap/heading", max_msg_size_);
    }

    void Publishers::publish_ins(const clap_b7_driver::msg::ClapIns &ins_msg) {
        if(ins_pub_){
            ins_pub_->publish(ins_msg);
        }
    }

    void Publishers::publish_autoware_orientation( const autoware_sensing_msgs::msg::GnssInsOrientationStamped &autoware_orientation_msg) {
        if(gnss_ins_orientation_pub_){
            gnss_ins_orientation_pub_->publish(autoware_orientation_msg);
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

    void Publishers::publish_twist(const geometry_msgs::msg::TwistWithCovarianceStamped& twist_msg){
        if(twist_pub_){
            twist_pub_->publish(twist_msg);
        }
    }
    void Publishers::publish_nav_sat_fix(const sensor_msgs::msg::NavSatFix& nav_sat_fix_msg){
        if(nav_sat_fix_pub_){
            nav_sat_fix_pub_->publish(nav_sat_fix_msg);
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

    void Publishers::publish_gnss_odom(const nav_msgs::msg::Odometry& gnss_odom_msg){
        if(gnss_odom_pub_){
            gnss_odom_pub_->publish(gnss_odom_msg);
        }
    }

    void Publishers::broadcast_transforms(const geometry_msgs::msg::TransformStamped& gnss_odom_tf_){
        if(tf_broadcaster_odom_){
            tf_broadcaster_odom_->sendTransform(gnss_odom_tf_);
        }
    }

    void Publishers::broadcast_static_transform(const geometry_msgs::msg::TransformStamped& gnss_odom_tf_){
        if(tf_static_broadcaster_){
            tf_static_broadcaster_->sendTransform(gnss_odom_tf_);
        }
    }

    void Publishers:: publish_ecef(const clap_b7_driver::msg::ClapECEF& ecef_msg){
        if(pub_ecef_) {
            pub_ecef_->publish(ecef_msg);
        }
    }

    void Publishers:: publish_twist_ecef(const geometry_msgs::msg::TwistWithCovarianceStamped& twist_msg){
        if(twist_pub_ecef) {
            twist_pub_ecef->publish(twist_msg);
        }
    }

    void Publishers:: publish_raw_navsatfix(const sensor_msgs::msg::NavSatFix& nav_sat_fix_msg){
        if(nav_sat_fix_raw_pub_) {
            nav_sat_fix_raw_pub_->publish(nav_sat_fix_msg);
        }
    }

    void Publishers:: publish_raw_imu(const sensor_msgs::msg::Imu& imu_msg){
        if(raw_imu_pub_) {
            raw_imu_pub_->publish(imu_msg);
        }
    }

    void Publishers:: publish_wheel_odom(const clap_b7_driver::msg::ClapWheelOdom &wheel_odom_msg){
        if(pub_wheel_odom_) {
            pub_wheel_odom_->publish(wheel_odom_msg);
        }
    }
} // namespace clap_b7