//
// Created by yelaydin on 07.07.2023.
//
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include <clap_b7_driver/clap_msg_wrapper.h>

#include <tf2/LinearMath/Quaternion.h>


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

    double ClapMsgWrapper:: raw_acc_to_m_s2(int32_t raw_acc) {
        return static_cast<double>(raw_acc) * accel_scale_factor * hz_to_second;
    }
    double ClapMsgWrapper::raw_gyro_to_deg_s(int32_t raw_gyro) {
        return static_cast<double>(raw_gyro) * gyro_scale_factor * hz_to_second;
    }

    double ClapMsgWrapper::degree2radian(double degree) {
        return degree * M_PI / 180.0;
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

    geometry_msgs::msg::TwistWithCovarianceStamped ClapMsgWrapper::create_twist_msg(const clap_b7::BestGnssVel& gnss_vel, float heading, int32_t z_gyro_raw, std::string frame_id) const{
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = create_header(std::move(frame_id));
        if(cos(degree2radian(heading - gnss_vel.track_angle)) < 0.0) {
            twist_msg.twist.twist.linear.x = -gnss_vel.horizontal_speed;
        }
        else{
            twist_msg.twist.twist.linear.x = gnss_vel.horizontal_speed;
        }

        twist_msg.twist.twist.linear.y = 0.0;
        twist_msg.twist.twist.linear.z = degree2radian(raw_gyro_to_deg_s(z_gyro_raw));

        twist_msg.twist.covariance[0] = 0.04;
        twist_msg.twist.covariance[7]  = 10000.0;
        twist_msg.twist.covariance[14] = 10000.0;
        twist_msg.twist.covariance[21] = 10000.0;
        twist_msg.twist.covariance[28] = 10000.0;
        twist_msg.twist.covariance[35] = 0.02;
        return twist_msg;
    }

    sensor_msgs::msg::NavSatFix ClapMsgWrapper::create_nav_sat_fix_msg(const clap_b7::InsPvax& ins, std::string frame_id) const {
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;

        nav_sat_fix_msg.header = create_header(std::move(frame_id));
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

    sensor_msgs::msg::NavSatFix ClapMsgWrapper::create_nav_sat_fix_msg(const clap_b7::BestGnssPos &gps_pos, std::string frame_id) const{
        sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
        nav_sat_fix_msg.header = create_header(std::move(frame_id));

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
        imu_msg.imu_temperature = static_cast<double>(((raw_imu.imu_status >> 16U) &  0x0000FFFFU) / 10.0);
        imu_msg.x_accel_output = raw_acc_to_m_s2(raw_imu.x_accel_output);
        imu_msg.y_accel_output = raw_acc_to_m_s2(raw_imu.y_accel_output);
        imu_msg.z_accel_output = raw_acc_to_m_s2(raw_imu.z_accel_output);

        imu_msg.x_gyro_output = degree2radian(raw_gyro_to_deg_s(raw_imu.x_gyro_output));
        imu_msg.y_gyro_output = degree2radian(raw_gyro_to_deg_s(raw_imu.y_gyro_output));
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
} // namespace clap_b7