//
// Created by elaydin on 07.07.2023.
//

#include <cmath>

#include <fcntl.h>

#include <clap_b7_driver/clap_b7_driver.hpp>
#include <clap_b7_driver/clap_config_params.h>

//deneme

namespace clap_b7{
    ClapB7Driver::ClapB7Driver(const rclcpp::NodeOptions &options) : Node("clap_b7_driver",options), updater_(this)
    {
        RCLCPP_INFO(this->get_logger(), "ClapB7Driver is starting");
        //
        // Get the ROS private nodeHandle, where the parameters are loaded from the launch file.
        //
        updater_.setHardwareID("ClapB7");
        updater_.add("ClapB7TimeSync", this, &ClapB7Driver::check_time_sync);

        load_parameters();
        if(params_.get_pub_custom_msgs()){
            publishers_.init_custom_msgs_publisher(*this);
        }

        if(params_.get_pub_std_msgs()){
            publishers_.init_std_msgs_publisher(*this,params_);
        }

        if(params_.get_sub_ntrip_msgs()){
            rtcm_sub_ = this->create_subscription<mavros_msgs::msg::RTCM>(params_.get_rtcm_topic(), 10,
                                                              std::bind(&ClapB7Driver::rtcm_callback, this,
                                                                        std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "RTCM subscriber is created %s", params_.get_rtcm_topic().c_str());
        }

        if(params_.get_use_odometry()){
            if(params_.get_use_local_origin())
            {
                ll_to_utm_transform_.set_origin(params_.get_lat_origin(), params_.get_long_origin(), params_.get_alt_origin());


                geometry_msgs::msg::Pose pos_msg;
                pos_msg.position.x = ll_to_utm_transform_.m_utm0_.easting;
                pos_msg.position.x = ll_to_utm_transform_.m_utm0_.northing;
                pos_msg.position.x = ll_to_utm_transform_.m_utm0_.altitude;

                auto msg = msg_wrapper_.create_transform(pos_msg, params_.get_odometry_frame(), "base_link");
                publishers_.broadcast_static_transform(msg);
            }
        }

        try_serial_connection(params_.get_serial_port(), params_.get_baudrate());
        parser_.set_receive_callback(std::bind(&ClapB7Driver::clap_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        msg_wrapper_.set_use_ros_time(params_.get_use_ros_time());
    }

    void ClapB7Driver::load_parameters(){
        rclcpp::NodeOptions node_opt;
        node_opt.automatically_declare_parameters_from_overrides(true);
        rclcpp::Node n_private("npv", "", node_opt);
        params_.load_parameters(n_private);
    }

    void ClapB7Driver::try_serial_connection(const std::basic_string<char>&port, unsigned int baud) {
        do {
            RCLCPP_INFO(this->get_logger(), "Trying to connect to serial port: %s", port.c_str());
            file_descriptor_ = open(port.c_str(), O_RDWR| O_NOCTTY | O_NONBLOCK);
            int opts = fcntl(file_descriptor_, F_GETFL);
            opts = opts & (O_NONBLOCK);
            fcntl(file_descriptor_, F_SETFL, opts);
        }while(file_descriptor_ == -1);

        memset(&tty_, 0, sizeof(tty_));
        if (tcgetattr(file_descriptor_, &tty_) != 0) {
            RCLCPP_INFO(this->get_logger(), "Error getting serial port attributes: %s", strerror(errno));
            close(file_descriptor_);
            return;
        }

        speed_t speed = B9600;
        switch (baud) {
            case 9600:
                speed = B9600;
                break;
            case 19200:
                speed = B19200;
                break;
            case 38400:
                speed = B38400;
                break;
            case 57600:
                speed = B57600;
                break;
            case 115200:
                speed = B115200;
                break;
            case 230400:
                speed = B230400;
                break;
            case 460800:
                speed = B460800;
                break;
            case 921600:
                speed = B500000;
                break;
            default:
                RCLCPP_INFO(this->get_logger(),"Unsupported baud rate");
                close(file_descriptor_);
        }
        cfsetospeed(&tty_, speed);
        cfsetispeed(&tty_, speed);

        tty_.c_cflag |= (CLOCAL | CREAD);
        tty_.c_cflag &= ~PARENB; // Disable parity
        tty_.c_cflag &= ~CSTOPB; // One stop bit
        tty_.c_cflag &= ~CSIZE;  // Clear data size bits
        tty_.c_cflag |= CS8;     // 8 bits per byte

        tty_.c_cc[VMIN] = 0;  // Minimum number of characters to read
        tty_.c_cc[VTIME] = 0; // Timeout in tenths of a second

        if (tcsetattr(file_descriptor_, TCSANOW, &tty_) != 0) {
            RCLCPP_INFO(this->get_logger(), "Error getting serial port attributes: %s", strerror(errno));
            close(file_descriptor_);
            return;
        }
    }

    void ClapB7Driver::Update(){
        unsigned char buffer[255];
        ssize_t len = read(file_descriptor_, buffer, sizeof(buffer));
        if (len > 0) {
            parser_.received_new_data(reinterpret_cast<const uint8_t*>(buffer), static_cast<uint16_t>(len));
        }
    }

    void ClapB7Driver::clap_read_callback(const uint8_t *data, uint16_t id) {
        msg_wrapper_.set_system_time(parser_.get_unix_time_ns());

        switch(static_cast<clap_b7::BinaryParser::MessageId>(id)) {
            case clap_b7::BinaryParser::MessageId::kWheelOdom : {
                std::memcpy(&wheel_data_, data, sizeof(TimeDWheelData));
                auto msg = msg_wrapper_.create_wheel_odom_msg(wheel_data_);
                publishers_.publish_wheel_odom(msg);
                break;
            }
            case clap_b7::BinaryParser::MessageId::kRAWIMU: {
                std::memcpy(&raw_imu_, data, sizeof(RawImu));

                auto msg = msg_wrapper_.create_imu_msg(raw_imu_, params_.get_imu_frame());
                publishers_.publish_adis16470_imu(msg);

                auto temp_msg = msg_wrapper_.create_temperature_msg(raw_imu_, params_.get_gnss_frame());
                publishers_.publish_temperature(temp_msg);

                auto imu_msg = msg_wrapper_.create_raw_imu_msg(raw_imu_, params_.get_imu_frame());
                if(!msg_wrapper_.is_ins_active(ins_pvax_)){
                    publishers_.publish_imu(imu_msg);
                }
                auto twist_msg = msg_wrapper_.create_twist_msg(gnss_vel_, heading_.heading, raw_imu_, params_.get_imu_frame());
                publishers_.publish_twist(twist_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kHeading: {
                std::memcpy(&heading_, data, sizeof(UniHeading));
                heading_.heading = static_cast<float>(ClapMsgWrapper::add_heading_offset(heading_.heading, params_.get_true_heading_offset()));
                auto msg = msg_wrapper_.create_gps_heading_msg(heading_, params_.get_gnss_frame());
                publishers_.publish_heading(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssPos: {
                std::memcpy(&gnss_pos_, data, sizeof(BestGnssPos));
                auto custom_msg = msg_wrapper_.create_gps_pos_msg(gnss_pos_, params_.get_gnss_frame());
                auto sensor_msg = msg_wrapper_.create_nav_sat_fix_msg(gnss_pos_, params_.get_gnss_frame(), params_.get_altitude_mode());
                if(!clap_b7::ClapMsgWrapper::is_ins_active(ins_pvax_)){
                    publishers_.publish_nav_sat_fix(sensor_msg);
                }
                publishers_.publish_raw_navsatfix(sensor_msg);
                publishers_.publish_gps_pos(custom_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssVel: {
                std::memcpy(&gnss_vel_, data, sizeof(BestGnssVel));

                auto custom_msg = msg_wrapper_.create_gps_vel_msg(gnss_vel_, params_.get_gnss_frame());
                publishers_.publish_gps_vel(custom_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kINSPVAX: {
                std::memcpy(&ins_pvax_, data, sizeof(InsPvax));
                if(clap_b7::ClapMsgWrapper::is_ins_active(ins_pvax_)){
                    auto std_msg = msg_wrapper_.create_nav_sat_fix_msg(ins_pvax_, params_.get_gnss_frame(), params_.get_altitude_mode());
                    publishers_.publish_nav_sat_fix(std_msg);
                    /*
                     * ODOM
                     */
                    if(params_.get_use_odometry()){
                        double x = NAN;
                        double y = NAN;
                        double z = NAN;
                        try{
                            if(params_.get_use_local_origin()){
                                ll_to_utm_transform_.transform_local(ins_pvax_.latitude, ins_pvax_.longitude, ins_pvax_.height, x, y, z);
                            }
                            else{
                                ll_to_utm_transform_.transform_global(ins_pvax_.latitude, ins_pvax_.longitude, ins_pvax_.height, x, y, z);
                            }

                        }
                        catch(std::runtime_error &exc){
                            RCLCPP_ERROR(this->get_logger(), "Could not transform from ll to utm(%s)", exc.what());
                        }
                        auto odom_msg = msg_wrapper_.create_odom_msg(ins_pvax_, raw_imu_, x, y, z, params_.get_odometry_frame(), "base_link");
                        publishers_.publish_gnss_odom(odom_msg);

                        auto pos_msg = msg_wrapper_.create_transform(odom_msg.pose.pose, odom_msg.header.frame_id, "base_link");
                        publishers_.broadcast_transforms(pos_msg);
                    }
                }

                if(msg_wrapper_.is_ins_active(ins_pvax_)){
                    auto msg = msg_wrapper_.create_sensor_imu_msg(raw_imu_, ins_pvax_, params_.get_imu_frame());
                    publishers_.publish_imu(msg);

                    auto autoware_msg = msg_wrapper_.create_autoware_orientation_msg(ins_pvax_, params_.get_gnss_frame());
                    publishers_.publish_autoware_orientation(autoware_msg);
                }

                auto custom_msg = msg_wrapper_.create_ins_msg(ins_pvax_, params_.get_gnss_frame());
                publishers_.publish_ins(custom_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kECEF: {
                std::memcpy(&ecef_, data, sizeof(ECEF));
                auto msg = msg_wrapper_.create_ecef_msg(ecef_);
                publishers_.publish_ecef(msg);
                auto twist_msg = msg_wrapper_.create_twist_msg(ecef_, raw_imu_, params_.get_imu_frame());
                publishers_.publish_twist_ecef(twist_msg);
                break;
            }
        }
    }

    void ClapB7Driver::rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg) {
        const char *start_ptr = reinterpret_cast<const char*>(msg->data.data());
        int size = write(file_descriptor_, start_ptr, msg->data.size());
        if(size != static_cast<int>(msg->data.size())){
            RCLCPP_ERROR(this->get_logger(), "RTCM data could not be sent");
        }
    }
    void ClapB7Driver::check_time_sync(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        diagnostic_msgs::msg::KeyValue key_value;
        if(msg_wrapper_.is_delay_high(parser_.get_unix_time_ns())){
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "GPS Time is not reliable ROS Time is used");
        }
        else{
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GPS Time is reliable");
        }
        key_value.key = "clap_timestamp";
        key_value.value = std::to_string(parser_.get_unix_time_ns() * 1e-9);
        stat.values.push_back(key_value);
        key_value.key = "ros_time";
        key_value.value=std::to_string(rclcpp::Clock().now().seconds());
        stat.values.push_back(key_value);
        key_value.key = "delay";
        key_value.value = std::to_string((rclcpp::Clock().now().nanoseconds() - parser_.get_unix_time_ns()) * 1e-6) + "ms";
        stat.values.push_back(key_value);
    }
} // namespace clap_b7
