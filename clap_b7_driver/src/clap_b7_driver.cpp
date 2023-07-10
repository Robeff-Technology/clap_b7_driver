//
// Created by elaydin on 07.07.2023.
//

#include <clap_b7_driver/clap_b7_driver.hpp>



namespace clap_b7{
    ClapB7Driver::ClapB7Driver() : Node("clap_b7_driver")
    {
        RCLCPP_INFO(this->get_logger(), "ClapB7Driver is starting");
        //
        // Get the ROS private nodeHandle, where the parameters are loaded from the launch file.
        //
        load_parameters();
        RCLCPP_ERROR(this->get_logger(), "Connected to serial port(%s) baudrate = %d\n", params_.get_serial_port().c_str(), params_.get_baudrate());
        std::string port{"/dev/ttyUSB0"};
        try_serial_connection(port, 460800);
        serial_.setCallback(std::bind(&ClapB7Driver::serial_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        parser_.set_receive_callback(std::bind(&ClapB7Driver::clap_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        /*
         * std msgs publishers
         */
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", 10);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist", 10);

        /*
         * clap msgs publishers
         */
        gps_pos_pub_ = this->create_publisher<clap_b7_driver::msg::ClapGpsPos>("clap_gnss_pos", 10);
        gps_vel_pub_ = this->create_publisher<clap_b7_driver::msg::ClapGpsVel>("clap_gnss_vel", 10);
        adis16470_imu_pub_ = this->create_publisher<clap_b7_driver::msg::ClapImu>("clap_adis16470", 10);
        ins_pub_ = this->create_publisher<clap_b7_driver::msg::ClapIns>("clap_ins", 10);


    }

    void ClapB7Driver::load_parameters(){
        rclcpp::NodeOptions node_opt;
        node_opt.automatically_declare_parameters_from_overrides(true);
        rclcpp::Node n_private("npv", "", node_opt);
        params_.load_parameters(n_private);
    }

    void ClapB7Driver::try_serial_connection(std::string &port, unsigned int baud) {
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
        switch(static_cast<clap_b7::BinaryParser::MessageId>(id)) {
            case clap_b7::BinaryParser::MessageId::kRAWIMU: {
                memcpy(&raw_imu_, data, sizeof(RawImu));

                auto msg = msg_wrapper_.create_imu_msg(raw_imu_, "adis16570_imu");
                adis16470_imu_pub_->publish(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kHeading: {
                memcpy(&heading_, data, sizeof(UniHeading));
                auto msg = msg_wrapper_.create_gps_heading_msg(heading_, "clap_heading");
                heading_pub_->publish(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssPos: {
                memcpy(&gnss_pos_, data, sizeof(BestGnssPos));
                sensor_msgs::msg::NavSatFix msg;
                if(clap_b7::ClapMsgWrapper::is_ins_active(ins_pvax_)){
                    msg = msg_wrapper_.create_nav_sat_fix_msg(ins_pvax_, "gnss");
                }
                else{
                    msg = msg_wrapper_.create_nav_sat_fix_msg(gnss_pos_, "gnss");
                }

                auto custom_msg = msg_wrapper_.create_gps_pos_msg(gnss_pos_, "clap_gnss_pos");
                gps_pos_pub_->publish(custom_msg);
                nav_sat_fix_pub_->publish(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssVel: {
                memcpy(&gnss_vel_, data, sizeof(BestGnssVel));
                auto msg = msg_wrapper_.create_twist_msg(gnss_vel_, heading_.heading, raw_imu_.z_gyro_output, "gnss");
                twist_pub_->publish(msg);

                auto custom_msg = msg_wrapper_.create_gps_vel_msg(gnss_vel_, "clap_gnss_vel");
                gps_vel_pub_->publish(custom_msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kINSPVAX: {
                memcpy(&ins_pvax_, data, sizeof(InsPvax));
                if(clap_b7::ClapMsgWrapper::is_ins_active(ins_pvax_)){
                    auto msg = msg_wrapper_.create_sensor_imu_msg(raw_imu_, ins_pvax_, "gnss");
                    imu_pub_->publish(msg);
                }
                auto custom_msg = msg_wrapper_.create_ins_msg(ins_pvax_, "clap_ins");
                ins_pub_->publish(custom_msg);
                break;
            }
        }
    }
    void ClapB7Driver::serial_read_callback(const char *data, size_t len) {
        parser_.received_new_data(reinterpret_cast<const uint8_t*>(data), static_cast<uint16_t>(len));
    }
} // namespace clap_b7
