//
// Created by elaydin on 07.07.2023.
//

#include <clap_b7_driver/clap_b7_driver.hpp>
#include <sensor_msgs/msg/imu.hpp>
namespace clap_b7{
    ClapB7Driver::ClapB7Driver() : Node("clap_b7_driver")
    {
        std::string port{"/dev/ttyUSB0"};
        try_serial_connection(port, 460800);
        serial_.setCallback(std::bind(&ClapB7Driver::serial_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        parser_.set_receive_callback(std::bind(&ClapB7Driver::clap_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", 10);
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
                break;
            }

            case clap_b7::BinaryParser::MessageId::kHeading: {
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssPos: {
                memcpy(&gnss_pos_, data, sizeof(BestGnssPos));
                sensor_msgs::msg::NavSatFix msg;
                if(msg_wrapper_.is_ins_active(ins_pvax_)){
                    RCLCPP_INFO(this->get_logger(), "INS is active");
                    msg = msg_wrapper_.create_nav_sat_fix_msg(ins_pvax_, "gnss");
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "INS is not active");
                    msg = msg_wrapper_.create_nav_sat_fix_msg(gnss_pos_, "gnss");
                }
                nav_sat_fix_pub_->publish(msg);
                break;
            }

            case clap_b7::BinaryParser::MessageId::kBestGnssVel: {
                break;
            }

            case clap_b7::BinaryParser::MessageId::kINSPVAX: {
                memcpy(&ins_pvax_, data, sizeof(InsPvax));
                if(msg_wrapper_.is_ins_active(ins_pvax_)){
                    auto msg = msg_wrapper_.create_sensor_imu_msg(raw_imu_, ins_pvax_, "gnss");
                    imu_pub_->publish(msg);
                }
                break;
            }
        }
    }
    void ClapB7Driver::serial_read_callback(const char *data, size_t len) {
        parser_.received_new_data(reinterpret_cast<const uint8_t*>(data), static_cast<uint16_t>(len));
    }
} // namespace clap_b7
