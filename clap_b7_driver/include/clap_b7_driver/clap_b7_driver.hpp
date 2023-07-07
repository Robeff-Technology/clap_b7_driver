//
// Created by elaydin on 07.07.2023.
//

#ifndef CLAP_B7_DRIVER_HPP
#define CLAP_B7_DRIVER_HPP
#include <rclcpp/rclcpp.hpp>
#include "AsyncSerial.h"
#include "clap_b7_driver/clap_binary_parser.h"
#include <clap_b7_driver/clap_msg_wrapper.h>
#include <clap_b7_driver/clap_structs.h>
namespace clap_b7
{
    class ClapB7Driver : public rclcpp::Node {
    public:
        ClapB7Driver();
        ~ClapB7Driver() override {
            serial_.close();
        }
    private:
        CallbackAsyncSerial serial_;
        void serial_read_callback(const char *data, size_t len);
        void try_serial_connection(std::string &port, unsigned int baud);
        clap_b7::BinaryParser parser_{};
        void clap_read_callback(const uint8_t *data, uint16_t id);
        rclcpp::TimerBase::SharedPtr timer_;
        ClapMsgWrapper msg_wrapper_{};
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;
        RawImu raw_imu_{};
        InsPvax ins_pvax_{};
        BestGnssPos gnss_pos_{};

    };
} // namespace clap_b7


#endif //CLAP_B7_DRIVER_HPP
