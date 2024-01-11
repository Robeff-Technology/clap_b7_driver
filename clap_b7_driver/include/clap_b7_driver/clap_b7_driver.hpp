//
// Created by elaydin on 07.07.2023.
//

#ifndef CLAP_B7_DRIVER_HPP
#define CLAP_B7_DRIVER_HPP
#include "AsyncSerial.h"

#include "clap_b7_driver/clap_binary_parser.h"
#include <clap_b7_driver/clap_msg_wrapper.h>
#include <clap_b7_driver/clap_structs.h>
#include <clap_b7_driver/clap_publisher.h>
#include <clap_b7_driver/ll_to_utm_transform.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <mavros_msgs/msg/rtcm.hpp>




namespace clap_b7
{
    class ClapB7Driver : public rclcpp::Node {
    public:
        ClapB7Driver(const rclcpp::NodeOptions &options);
        ~ClapB7Driver() override {
            serial_.close();
        }
    private:
        CallbackAsyncSerial serial_;
        void serial_read_callback(const char *data, size_t len);
        void try_serial_connection(const std::basic_string<char>& port, unsigned int baud);
        clap_b7::BinaryParser parser_{};
        void clap_read_callback(const uint8_t *data, uint16_t id);
        rclcpp::TimerBase::SharedPtr timer_;
        ClapMsgWrapper msg_wrapper_{};
        Publishers publishers_{};
        LlToUtmTransform ll_to_utm_transform_{};

        /*
         * Messages
         */
        RawImu raw_imu_{};
        InsPvax ins_pvax_{};
        BestGnssPos gnss_pos_{};
        BestGnssVel gnss_vel_{};
        UniHeading heading_{};
        ECEF ecef_{};
        TimeDWheelData wheel_data_{};

        /*
         * Config
         */
        ConfigParams params_;

        /*
         * Subscribers
         */

        rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr rtcm_sub_;
        void load_parameters();

        void rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg);

        // diagnostics
        diagnostic_updater::Updater updater_;
        void check_time_sync(diagnostic_updater::DiagnosticStatusWrapper &stat);
    };
} // namespace clap_b7


#endif //CLAP_B7_DRIVER_HPP
