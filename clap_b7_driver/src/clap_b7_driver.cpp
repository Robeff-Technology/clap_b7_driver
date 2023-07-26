//
// Created by elaydin on 07.07.2023.
//

#include <cmath>

#include <clap_b7_driver/clap_b7_driver.hpp>
#include <clap_b7_driver/clap_config_params.h>



namespace clap_b7{
    ClapB7Driver::ClapB7Driver() : Node("clap_b7_driver")
    {
        RCLCPP_INFO(this->get_logger(), "ClapB7Driver is starting");
        //
        // Get the ROS private nodeHandle, where the parameters are loaded from the launch file.
        //
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
            ll_to_utm_transform_.set_origin(params_.get_lat_origin(), params_.get_long_origin(), params_.get_alt_origin());


            geometry_msgs::msg::Pose pos_msg;
            pos_msg.position.x = ll_to_utm_transform_.m_utm0_.easting;
            pos_msg.position.x = ll_to_utm_transform_.m_utm0_.northing;
            pos_msg.position.x = ll_to_utm_transform_.m_utm0_.altitude;

            auto msg = msg_wrapper_.create_transform(pos_msg, params_.get_odometry_frame(), "base_link");
            publishers_.broadcast_static_transform(msg);
        }

        try_serial_connection(params_.get_serial_port(), params_.get_baudrate());
        serial_.setCallback(std::bind(&ClapB7Driver::serial_read_callback, this, std::placeholders::_1, std::placeholders::_2));
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
        // try to connect serial
        do {
            RCLCPP_INFO(this->get_logger(), "Trying to connect to serial port");
            try {
                serial_.open(port, baud);
            }
            catch(boost::system::system_error &exc){
                RCLCPP_ERROR(this->get_logger(), "Could not connect to serial port(%s)", exc.what());
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }while(!serial_.isOpen());

        RCLCPP_INFO(this->get_logger(), "Connected to serial port(%s)", port.c_str());
    }

    void ClapB7Driver::clap_read_callback(const uint8_t *data, uint16_t id) {
        msg_wrapper_.set_system_time(parser_.gnss_unix_time_ns_);
        switch(static_cast<clap_b7::BinaryParser::MessageId>(id)) {
            case clap_b7::BinaryParser::MessageId::kRAWIMU: {
                memcpy(&raw_imu_, data, sizeof(RawImu));

                auto msg = msg_wrapper_.create_imu_msg(raw_imu_, params_.get_gnss_frame());
                publishers_.publish_adis16470_imu(msg);

                auto temp_msg = msg_wrapper_.create_temperature_msg(raw_imu_, params_.get_gnss_frame());
                publishers_.publish_temperature(temp_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kHeading: {
                memcpy(&heading_, data, sizeof(UniHeading));
                heading_.heading = static_cast<float>(ClapMsgWrapper::add_heading_offset(heading_.heading, params_.get_true_heading_offset()));
                auto msg = msg_wrapper_.create_gps_heading_msg(heading_, params_.get_gnss_frame());
                publishers_.publish_heading(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssPos: {
                memcpy(&gnss_pos_, data, sizeof(BestGnssPos));
                sensor_msgs::msg::NavSatFix msg;
                if(clap_b7::ClapMsgWrapper::is_ins_active(ins_pvax_)){
                    msg = msg_wrapper_.create_nav_sat_fix_msg(ins_pvax_, params_.get_gnss_frame());
                }
                else{
                    msg = msg_wrapper_.create_nav_sat_fix_msg(gnss_pos_, params_.get_gnss_frame());
                }

                auto custom_msg = msg_wrapper_.create_gps_pos_msg(gnss_pos_, params_.get_gnss_frame());
                publishers_.publish_gps_pos(custom_msg);
                publishers_.publish_nav_sat_fix(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssVel: {
                memcpy(&gnss_vel_, data, sizeof(BestGnssVel));

                auto custom_msg = msg_wrapper_.create_gps_vel_msg(gnss_vel_, params_.get_gnss_frame());
                publishers_.publish_gps_vel(custom_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kINSPVAX: {
                memcpy(&ins_pvax_, data, sizeof(InsPvax));
                if(clap_b7::ClapMsgWrapper::is_ins_active(ins_pvax_)){
                    auto msg = msg_wrapper_.create_sensor_imu_msg(raw_imu_, ins_pvax_, params_.get_gnss_frame());
                    publishers_.publish_imu(msg);
                    /*
                     * ODOM
                     */
                    if(params_.get_use_odometry()){
                        double x = NAN;
                        double y = NAN;
                        double z = NAN;
                        try{
                            ll_to_utm_transform_.transform(ins_pvax_.latitude, ins_pvax_.longitude, ins_pvax_.height, x, y, z);
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
                auto custom_msg = msg_wrapper_.create_ins_msg(ins_pvax_, params_.get_gnss_frame());
                publishers_.publish_ins(custom_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kECEF: {
                memcpy(&ecef_, data, sizeof(ECEF));
                auto msg = msg_wrapper_.create_ecef_msg(ecef_);
                publishers_.publish_ecef(msg);
                auto twist_msg = msg_wrapper_.create_twist_msg(ecef_, raw_imu_, params_.get_gnss_frame());
                publishers_.publish_twist_ecef(twist_msg);
                break;
            }
        }
    }
    void ClapB7Driver::serial_read_callback(const char *data, size_t len) {
        parser_.received_new_data(reinterpret_cast<const uint8_t*>(data), static_cast<uint16_t>(len));
    }

    void ClapB7Driver::rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg) {
        const char *start_ptr = reinterpret_cast<const char*>(msg->data.data());
        serial_.write(start_ptr, msg->data.size());
        RCLCPP_INFO(this->get_logger(), "RTCM data received");
    }
} // namespace clap_b7
