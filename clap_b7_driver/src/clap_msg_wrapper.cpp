//
// Created by yelaydin on 07.07.2023.
//
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include <clap_b7_driver/clap_msg_wrapper.h>

#include <tf2/LinearMath/Quaternion.h>

#include <GeographicLib/UTMUPS.hpp>



constexpr double accel_scale_factor = 0.000000186;
constexpr double hz_to_second = 100;
constexpr double gyro_scale_factor = 0.000001006;
namespace clap_b7{
    ClapMsgWrapper::~ClapMsgWrapper() = default;
    void ClapMsgWrapper::set_system_time(int64_t timestamp) {
        clap_timestamp = timestamp;
    }

    bool ClapMsgWrapper::is_ins_active(const clap_b7::InsPvax& ins){
        return ins.ins_status > 1;
    }

    bool ClapMsgWrapper::is_ins_initialized(const clap_b7::InsPvax& ins){
        if(ins.ins_status > 1)
            ins_initialized_ = true;
        return ins_initialized_;
    }

    double ClapMsgWrapper::calc_imu_temperature(const clap_b7::RawImu &raw_imu) {
        auto raw_temperature = static_cast<int16_t>((raw_imu.imu_status >> 16U) & 0xFFFFU);
        return static_cast<double>(raw_temperature) * 0.1;
    }

    double ClapMsgWrapper:: raw_acc_to_m_s2(int32_t raw_acc) {
        return static_cast<double>(raw_acc) * accel_scale_factor * hz_to_second;
    }
    double ClapMsgWrapper::raw_gyro_to_deg_s(int32_t raw_gyro) {
        return static_cast<double>(raw_gyro) * gyro_scale_factor * hz_to_second;
    }

    double ClapMsgWrapper::raw_gyro_to_deg(int32_t raw_gyro) {
        return static_cast<double>(raw_gyro) * gyro_scale_factor;
    }


    double ClapMsgWrapper::degree2radian(double degree) {
        return degree * M_PI / 180.0;
    }

    Eigen::Matrix3d ClapMsgWrapper::convert_stddev_llh_to_enu(const Eigen::Matrix3d& covarianceLLH, const Eigen::Matrix3d& rotationMatrix) {
        Eigen::Matrix3d covariance_enu = rotationMatrix * covarianceLLH * rotationMatrix.transpose();
        return covariance_enu;
    }

    std_msgs::msg::Header ClapMsgWrapper::create_header(std::string frame_id) const {
        std_msgs::msg::Header header;
        if(use_ros_time_) {
            header.stamp = rclcpp::Clock().now();
        }
        else {
            header.stamp = rclcpp::Time(clap_timestamp);
        }
        header.frame_id = std::move(frame_id);
        return header;
    }

    sensor_msgs::msg::NavSatFix ClapMsgWrapper::create_nav_sat_fix_msg(const clap_b7::InsPvax& ins, std::string frame_id, int alt_mode) const {
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;

        nav_sat_fix_msg.header = create_header(std::move(frame_id));

        if(ins.pos_type == 56 || ins.pos_type == 55) {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        else if(ins.pos_type == 54) {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        }
        else if(ins.pos_type < 54 && ins.pos_type >= 52){
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        }
        else {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }

        nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        nav_sat_fix_msg.latitude = ins.latitude;
        nav_sat_fix_msg.longitude = ins.longitude;
        if(alt_mode){
            nav_sat_fix_msg.altitude = ins.height + ins.undulation;
        }
        else{
            nav_sat_fix_msg.altitude = ins.height;
        }


        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        Eigen::Matrix3d covariance_llh;
        covariance_llh << ins.std_dev_latitude * ins.std_dev_latitude, 0.0, 0.0,
                          0.0, ins.std_dev_longitude * ins.std_dev_longitude, 0.0,
                          0.0, 0.0, ins.std_dev_height * ins.std_dev_height;

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << 0.0, 1.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 0.0, -1.0;

        Eigen::Matrix3d covariance_enu = convert_stddev_llh_to_enu(covariance_llh, rotation_matrix);

        nav_sat_fix_msg.position_covariance[0] = covariance_enu(0, 0);
        nav_sat_fix_msg.position_covariance[4] = covariance_enu(1, 1);
        nav_sat_fix_msg.position_covariance[8] = covariance_enu(2, 2);


        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return nav_sat_fix_msg;
    }

    sensor_msgs::msg::NavSatFix ClapMsgWrapper::create_nav_sat_fix_msg(const clap_b7::BestGnssPos &gps_pos, std::string frame_id, int alt_mode) const{
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
        nav_sat_fix_msg.header = create_header(std::move(frame_id));

        if(gps_pos.pos_type >= 32) {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        else if(gps_pos.pos_type == 18){
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        }
        else if(gps_pos.pos_type == 16){
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        }
        else {
            nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }



        nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        nav_sat_fix_msg.latitude = gps_pos.latitude;
        nav_sat_fix_msg.longitude = gps_pos.longitude;
        if(alt_mode){
            nav_sat_fix_msg.altitude = gps_pos.height + gps_pos.undulation;
        }
        else{
            nav_sat_fix_msg.altitude = gps_pos.height;
        }

        Eigen::Matrix3d covariance_llh;
        covariance_llh << gps_pos.std_dev_latitude * gps_pos.std_dev_latitude, 0.0, 0.0,
                0.0, gps_pos.std_dev_longitude * gps_pos.std_dev_longitude, 0.0,
                0.0, 0.0, gps_pos.std_dev_height * gps_pos.std_dev_height;

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0;

        Eigen::Matrix3d covariance_enu = convert_stddev_llh_to_enu(covariance_llh, rotation_matrix);

        nav_sat_fix_msg.position_covariance[0] = covariance_enu(0, 0);
        nav_sat_fix_msg.position_covariance[4] = covariance_enu(1, 1);
        nav_sat_fix_msg.position_covariance[8] = covariance_enu(2, 2);

        nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        return nav_sat_fix_msg;
    }

    sensor_msgs::msg::Imu ClapMsgWrapper::create_sensor_imu_msg(const clap_b7::RawImu& raw_imu, const clap_b7::InsPvax& ins, std::string frame_id) const {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = create_header(std::move(frame_id));
        tf2::Quaternion q;
        /*
         * in clap b7 roll-> y-axis pitch-> x axis azimuth->left-handed rotation around z-axis
         * in ros imu msg roll-> x-axis pitch-> y axis azimuth->right-handed rotation around z-axis
         */
        q.setRPY(degree2radian(ins.pitch), degree2radian(ins.roll), degree2radian(-ins.azimuth));
        imu_msg.orientation.w = q.getW();
        imu_msg.orientation.x = q.getX();
        imu_msg.orientation.y = q.getY();
        imu_msg.orientation.z = q.getZ();

        imu_msg.angular_velocity.x = degree2radian(raw_gyro_to_deg_s(raw_imu.x_gyro_output));
        imu_msg.angular_velocity.y = -1.0 * degree2radian(raw_gyro_to_deg_s(raw_imu.y_gyro_output));
        imu_msg.angular_velocity.z = degree2radian(raw_gyro_to_deg_s(raw_imu.z_gyro_output));

        imu_msg.linear_acceleration.x = raw_acc_to_m_s2(raw_imu.x_accel_output);
        imu_msg.linear_acceleration.y = -1.0 * raw_acc_to_m_s2(raw_imu.y_accel_output);
        imu_msg.linear_acceleration.z = raw_acc_to_m_s2(raw_imu.z_accel_output);

        imu_msg.orientation_covariance[0] = ins.std_dev_pitch * ins.std_dev_pitch;
        imu_msg.orientation_covariance[4] = ins.std_dev_roll * ins.std_dev_roll;
        imu_msg.orientation_covariance[8] = ins.std_dev_azimuth * ins.std_dev_azimuth;

        /*
         * angular velocity and linear acceleration covariance is not provided by clap b7
         */
        for(size_t i = 0; i < 9; i += 4){
            imu_msg.angular_velocity_covariance[i] = 0.00;
            imu_msg.linear_acceleration_covariance[i] = 0.00;
        }

        return imu_msg;
    }

    void ClapMsgWrapper::set_use_ros_time(bool use_ros_time) {
        use_ros_time_ = use_ros_time;
    }

   clap_b7_driver::msg::ClapGpsPos ClapMsgWrapper::create_gps_pos_msg(const clap_b7::BestGnssPos& gnss_pos, std::string frame_id) const{
        clap_b7_driver::msg::ClapGpsPos gps_pos_msg;
        gps_pos_msg.header = create_header(std::move(frame_id));
        gps_pos_msg.latitude = gnss_pos.latitude;
        gps_pos_msg.longitude = gnss_pos.longitude;
        gps_pos_msg.altitude = gnss_pos.height;
        gps_pos_msg.undulation = gnss_pos.undulation;

        gps_pos_msg.std_dev_longitude = gnss_pos.std_dev_longitude;
        gps_pos_msg.std_dev_latitude = gnss_pos.std_dev_latitude;
        gps_pos_msg.std_dev_altitude = gnss_pos.std_dev_height;

        gps_pos_msg.number_of_satellite_tracked = gnss_pos.num_sats_tracked;
        gps_pos_msg.number_of_satellite_used = gnss_pos.num_sats_in_solution;
        gps_pos_msg.diff_age = gnss_pos.diff_age;

        gps_pos_msg.pos_type = gnss_pos.pos_type;
        gps_pos_msg.sol_status = gnss_pos.sol_status;
        return gps_pos_msg;
    }

    clap_b7_driver::msg::ClapGpsVel ClapMsgWrapper::create_gps_vel_msg(const clap_b7::BestGnssVel& gnss_vel, std::string frame_id)const{
        clap_b7_driver::msg::ClapGpsVel gps_vel_msg;
        gps_vel_msg.header = create_header(std::move(frame_id));

        gps_vel_msg.sol_status = gnss_vel.sol_status;
        gps_vel_msg.vel_type = gnss_vel.vel_type;
        gps_vel_msg.latency = gnss_vel.latency;
        gps_vel_msg.diff_age = gnss_vel.age;

        gps_vel_msg.hor_spd = gnss_vel.horizontal_speed;
        gps_vel_msg.trk_gnd = gnss_vel.track_angle;
        gps_vel_msg.vert_spd = gnss_vel.vertical_speed;

        return gps_vel_msg;
    }

    clap_b7_driver::msg::ClapHeading ClapMsgWrapper::create_gps_heading_msg(const clap_b7::UniHeading& uniheading, std::string frame_id) const{
        clap_b7_driver::msg::ClapHeading heading;
        heading.header = create_header(std::move(frame_id));

        heading.status = uniheading.sol_status;
        heading.heading = uniheading.heading;
        heading.std_dev_heading = uniheading.std_dev_heading;
        heading.pitch = uniheading.pitch;
        heading.std_dev_pitch = uniheading.std_dev_pitch;
        heading.baseline_len = uniheading.baseline_length;
        return heading;
    }

    clap_b7_driver::msg::ClapImu ClapMsgWrapper::create_imu_msg(const clap_b7::RawImu& raw_imu, std::string frame_id) const{
        clap_b7_driver::msg::ClapImu imu_msg;
        imu_msg.header = create_header(std::move(frame_id));

        imu_msg.imu_status = raw_imu.imu_status & 0x0000FFFFU;
        imu_msg.imu_temperature = calc_imu_temperature(raw_imu);
        imu_msg.x_accel_output = raw_acc_to_m_s2(raw_imu.x_accel_output);
        imu_msg.y_accel_output = -1.0 * raw_acc_to_m_s2(raw_imu.y_accel_output);
        imu_msg.z_accel_output = raw_acc_to_m_s2(raw_imu.z_accel_output);

        imu_msg.x_gyro_output = degree2radian(raw_gyro_to_deg_s(raw_imu.x_gyro_output));
        imu_msg.y_gyro_output = -1.0 * degree2radian(raw_gyro_to_deg_s(raw_imu.y_gyro_output));
        imu_msg.z_gyro_output = degree2radian(raw_gyro_to_deg_s(raw_imu.z_gyro_output));

        return imu_msg;
    }
    clap_b7_driver::msg::ClapIns ClapMsgWrapper::create_ins_msg(const clap_b7::InsPvax& ins, std::string frame_id)const{
        clap_b7_driver::msg::ClapIns ins_msg;
        ins_msg.header = create_header(std::move(frame_id));
        ins_msg.ins_status = ins.ins_status;
        ins_msg.pos_type = ins.pos_type;
        ins_msg.latitude = ins.latitude;
        ins_msg.longitude = ins.longitude;
        ins_msg.height = ins.height;
        ins_msg.undulation = ins.undulation;
        ins_msg.north_velocity = ins.north_velocity;
        ins_msg.east_velocity = ins.east_velocity;
        ins_msg.up_velocity = ins.up_velocity;
        ins_msg.roll = ins.roll;
        ins_msg.pitch = ins.pitch;
        ins_msg.azimuth = ins.azimuth;
        ins_msg.std_dev_latitude = ins.std_dev_latitude;
        ins_msg.std_dev_longitude = ins.std_dev_longitude;
        ins_msg.std_dev_height = ins.std_dev_height;
        ins_msg.std_dev_north_velocity = ins.std_dev_north_velocity;
        ins_msg.std_dev_east_velocity = ins.std_dev_east_velocity;
        ins_msg.std_dev_up_velocity = ins.std_dev_up_velocity;
        ins_msg.std_dev_roll = ins.std_dev_roll;
        ins_msg.std_dev_pitch = ins.std_dev_pitch;
        ins_msg.std_dev_azimuth = ins.std_dev_azimuth;
        ins_msg.extended_solution_stat = ins.extended_solution_stat;
        ins_msg.time_since_update = ins.time_since_update;

        return ins_msg;
    }

    sensor_msgs::msg::Temperature ClapMsgWrapper::create_temperature_msg(const RawImu &raw_imu, std::string frame_id) const {
        sensor_msgs::msg::Temperature temperature_msg;
        temperature_msg.header = create_header(std::move(frame_id));
        temperature_msg.temperature = calc_imu_temperature(raw_imu);
        return temperature_msg;
    }

    double ClapMsgWrapper :: add_heading_offset(double heading, double offset) {
        double heading_offset = heading + offset;
        if (heading_offset < 0.0) {
            heading_offset += 360.0;
        }
        return heading_offset;
    }

    nav_msgs::msg::Odometry ClapMsgWrapper::create_odom_msg(const InsPvax& ins, const RawImu& imu, double x, double y, double z, std::string frame_id, std::string child_frame_id) const {

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header = create_header(std::move(frame_id));

        odom_msg.child_frame_id = std::move(child_frame_id);

        tf2::Quaternion q;

        /*
        * in clap b7 roll-> y-axis pitch-> x axis azimuth->left-handed rotation around z-axis
        * in ros imu msg roll-> x-axis pitch-> y axis azimuth->right-handed rotation around z-axis
        */
        q.setRPY(degree2radian(ins.pitch), degree2radian(ins.roll), degree2radian(-ins.azimuth));

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();


        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        odom_msg.twist.twist.linear.x      = ins.east_velocity;
        odom_msg.twist.twist.linear.y      = ins.north_velocity;
        odom_msg.twist.twist.linear.z      = ins.up_velocity;

        odom_msg.twist.twist.angular.x     = degree2radian(raw_gyro_to_deg_s(imu.x_gyro_output));
        odom_msg.twist.twist.angular.y     = -1.0 * degree2radian(raw_gyro_to_deg_s(imu.y_gyro_output));
        odom_msg.twist.twist.angular.z     = degree2radian(raw_gyro_to_deg_s(imu.z_gyro_output));

        odom_msg.twist.covariance[0*6 + 0] = ins.std_dev_east_velocity * ins.std_dev_east_velocity;
        odom_msg.twist.covariance[1*6 + 1] = ins.std_dev_north_velocity * ins.std_dev_north_velocity;
        odom_msg.twist.covariance[2*6 + 2] = ins.std_dev_up_velocity * ins.std_dev_up_velocity;
        odom_msg.twist.covariance[3*6 + 3] = 0.01;
        odom_msg.twist.covariance[4*6 + 4] = 0.01;
        odom_msg.twist.covariance[5*6 + 5] = 0.01;

        return odom_msg;
    }

    geometry_msgs::msg::TransformStamped ClapMsgWrapper::create_transform(const geometry_msgs::msg::Pose &ref_pose, std::string frame_id, std::string child_frame_id) const{
        geometry_msgs::msg::TransformStamped transform;
        transform.header = create_header(std::move(frame_id));
        transform.child_frame_id = std::move(child_frame_id);
        transform.transform.translation.x = ref_pose.position.x;
        transform.transform.translation.y = ref_pose.position.y;
        transform.transform.translation.z = ref_pose.position.z;

        transform.transform.rotation.x = ref_pose.orientation.x;
        transform.transform.rotation.y = ref_pose.orientation.y;
        transform.transform.rotation.z = ref_pose.orientation.z;
        transform.transform.rotation.w = ref_pose.orientation.w;
        return transform;
    }

    clap_b7_driver::msg::ClapECEF ClapMsgWrapper::create_ecef_msg(const ECEF & ecef) const{
        clap_b7_driver::msg::ClapECEF ecef_msg;
        ecef_msg.pos_x = ecef.pos_x;
        ecef_msg.pos_y = ecef.pos_y;
        ecef_msg.pos_z = ecef.pos_z;

        ecef_msg.pos_type = ecef.pos_type;
        ecef_msg.sol_status = ecef.sol_status;

        ecef_msg.std_pos_x = ecef.std_pos_x;
        ecef_msg.std_pos_y = ecef.std_pos_y;
        ecef_msg.std_pos_z = ecef.std_pos_z;
        ecef_msg.v_sol_status = ecef.v_sol_status;
        ecef_msg.vel_type = ecef.vel_type;
        ecef_msg.vel_x = ecef.vel_x;
        ecef_msg.vel_y = ecef.vel_y;
        ecef_msg.vel_z = ecef.vel_z;
        ecef_msg.std_vel_x = ecef.std_vel_x;
        ecef_msg.std_vel_y = ecef.std_vel_y;
        ecef_msg.std_vel_z = ecef.std_vel_z;
        return ecef_msg;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped ClapMsgWrapper::create_twist_msg(const clap_b7::BestGnssVel& gnss_vel, float heading, const clap_b7::RawImu& imu, std::string frame_id) const{
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = create_header(std::move(frame_id));
        if(cos(degree2radian(heading - gnss_vel.track_angle)) < 0.0) {
            twist_msg.twist.twist.linear.x = gnss_vel.horizontal_speed;
        }
        else{
            twist_msg.twist.twist.linear.x = -gnss_vel.horizontal_speed;
        }

        twist_msg.twist.twist.linear.y = 0.0;
        twist_msg.twist.twist.linear.z = 0.0;

        twist_msg.twist.twist.angular.z = degree2radian(raw_gyro_to_deg_s(imu.z_gyro_output));
        twist_msg.twist.twist.angular.y = degree2radian(raw_gyro_to_deg_s(imu.y_gyro_output));
        twist_msg.twist.twist.angular.x = degree2radian(raw_gyro_to_deg_s(imu.x_gyro_output));

        twist_msg.twist.covariance[0] = 0.04;
        twist_msg.twist.covariance[7]  = 10000.0;
        twist_msg.twist.covariance[14] = 10000.0;
        twist_msg.twist.covariance[21] = 10000.0;
        twist_msg.twist.covariance[28] = 10000.0;
        twist_msg.twist.covariance[35] = 0.02;
        return twist_msg;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped ClapMsgWrapper::create_twist_msg(const ECEF & ecef, const RawImu& imu, std::string frame_id) const{
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = create_header(std::move(frame_id));

        twist_msg.twist.twist.linear.x = ecef.vel_x;
        twist_msg.twist.twist.linear.y = ecef.vel_y;
        twist_msg.twist.twist.linear.z = ecef.vel_z;

        twist_msg.twist.twist.angular.z = degree2radian(raw_gyro_to_deg_s(imu.z_gyro_output));
        twist_msg.twist.twist.angular.y = degree2radian(raw_gyro_to_deg_s(imu.y_gyro_output));
        twist_msg.twist.twist.angular.x = degree2radian(raw_gyro_to_deg_s(imu.x_gyro_output));

        twist_msg.twist.covariance[0] = ecef.std_vel_x * ecef.std_vel_x;
        twist_msg.twist.covariance[7]  = ecef.std_vel_y * ecef.std_vel_y;
        twist_msg.twist.covariance[14] = ecef.std_vel_z * ecef.std_vel_z;
        twist_msg.twist.covariance[21] = 0.0;
        twist_msg.twist.covariance[28] = 0.0;
        twist_msg.twist.covariance[35] = 0.0;
        return twist_msg;
    }

    double ClapMsgWrapper::scale_angle(double angle){
        double scaled_angle = angle;
        if(angle < 0.0){
            scaled_angle = 360.0 + angle;
        }
        else if(angle > 360.0){
            scaled_angle = angle - 360.0;
        }
        return scaled_angle;
    }
    sensor_msgs::msg::Imu ClapMsgWrapper::create_raw_imu_msg(const RawImu &imu, std::string frame_id) const{
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = create_header(std::move(frame_id));
        tf2::Quaternion q;
        static double roll_gyro = 0.0;
        static double pitch_gyro = 0.0;
        static double yaw_gyro = 0.0;

        double roll_acc = atan2(raw_acc_to_m_s2(imu.y_accel_output), raw_acc_to_m_s2(imu.z_accel_output));
        double pitch_acc = atan2(-raw_acc_to_m_s2(imu.x_accel_output), sqrt(raw_acc_to_m_s2(imu.y_accel_output) * raw_acc_to_m_s2(imu.y_accel_output) + raw_acc_to_m_s2(imu.z_accel_output) * raw_acc_to_m_s2(imu.z_accel_output)));

        roll_gyro += raw_gyro_to_deg(imu.x_gyro_output);
        pitch_gyro += raw_gyro_to_deg(imu.y_gyro_output);
        yaw_gyro += raw_gyro_to_deg(imu.z_gyro_output);


        double roll = roll_acc * 0.02 + roll_gyro * 0.98;
        double pitch = pitch_acc * 0.02 + pitch_gyro * 0.98;
        double yaw = yaw_gyro;

        q.setRPY(degree2radian(scale_angle(roll)), degree2radian(scale_angle(pitch)), degree2radian(scale_angle(yaw)));

        /*
         * in clap b7     roll->y-axis     pitch-> x axis     azimuth->left-handed rotation around z-axis
         * in ros imu msg roll-> x-axis    pitch-> y axis     azimuth->right-handed rotation around z-axis
         */
        //q.setRPY(degree2radian(ins.pitch), degree2radian(ins.roll), degree2radian(-ins.azimuth));
        imu_msg.orientation.w = q.getW();
        imu_msg.orientation.x = q.getX();
        imu_msg.orientation.y = q.getY();
        imu_msg.orientation.z = q.getZ();

        imu_msg.angular_velocity.x = degree2radian(raw_gyro_to_deg_s(imu.x_gyro_output));
        imu_msg.angular_velocity.y = -1.0 * degree2radian(raw_gyro_to_deg_s(imu.y_gyro_output));
        imu_msg.angular_velocity.z = degree2radian(raw_gyro_to_deg_s(imu.z_gyro_output));

        imu_msg.linear_acceleration.x = raw_acc_to_m_s2(imu.x_accel_output);
        imu_msg.linear_acceleration.y = -1.0 * raw_acc_to_m_s2(imu.y_accel_output);
        imu_msg.linear_acceleration.z = raw_acc_to_m_s2(imu.z_accel_output);

        imu_msg.orientation_covariance[0] = 1000.0;
        imu_msg.orientation_covariance[4] = 1000.0;
        imu_msg.orientation_covariance[8] = 1000.0;

        /*
         * angular velocity and linear acceleration covariance is not provided by clap b7
         */
        for(size_t i = 0; i < 9; i += 4){
            imu_msg.angular_velocity_covariance[i] = 0.001;
            imu_msg.linear_acceleration_covariance[i] = 0.001;
        }
        return imu_msg;
    }
    clap_b7_driver::msg::ClapWheelOdom ClapMsgWrapper::create_wheel_odom_msg(const TimeDWheelData &wheel_odom) const{
        static int32_t prev_ticks = 0;
        clap_b7_driver::msg::ClapWheelOdom wheel_odom_msg;
        wheel_odom_msg.ticks_per_rev = wheel_odom.cumulative_ticks - prev_ticks;
        wheel_odom_msg.wheel_speed = static_cast<uint16_t>(wheel_odom_msg.ticks_per_rev / 4);
        wheel_odom_msg.f_wheel_speed = wheel_odom.f_wheel_speed;
        wheel_odom_msg.direction = wheel_odom.direction;
        wheel_odom_msg.cumulative_ticks = wheel_odom.cumulative_ticks;
        prev_ticks = wheel_odom.cumulative_ticks;
        return wheel_odom_msg;
    }

    autoware_sensing_msgs::msg::GnssInsOrientationStamped ClapMsgWrapper::create_autoware_orientation_msg(const InsPvax &ins, const UniHeading& heading, std::string frame_id) const {
        autoware_sensing_msgs::msg::GnssInsOrientationStamped orientation_msg;
        orientation_msg.header = create_header(std::move(frame_id));
        tf2::Quaternion q;

        /*
        * in clap b7 roll-> y-axis pitch-> x axis azimuth->left-handed rotation around z-axis
        * in ros imu msg roll-> x-axis pitch-> y axis azimuth->right-handed rotation around z-axis
        */
        q.setRPY(degree2radian(heading.pitch), degree2radian(ins.roll), degree2radian(-heading.heading));

        orientation_msg.orientation.orientation.x = q.x();
        orientation_msg.orientation.orientation.y = q.y();
        orientation_msg.orientation.orientation.z = q.z();
        orientation_msg.orientation.orientation.w = q.w();


        orientation_msg.orientation.rmse_rotation_x = heading.std_dev_pitch * heading.std_dev_pitch;
        orientation_msg.orientation.rmse_rotation_y = ins.std_dev_roll * ins.std_dev_roll;
        orientation_msg.orientation.rmse_rotation_z = heading.std_dev_heading * heading.std_dev_heading;

        return orientation_msg;
    }
} // namespace clap_b7