//
// Created by elaydin on 25.07.2023.
//

#ifndef CONFIG_CLAP_B7_H
#define CONFIG_CLAP_B7_H

#include <rclcpp/rclcpp.hpp>
#include <clap_b7_driver/AsyncSerial.h>
#include <vector>
#include <boost/format.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
namespace clap_b7{

    class ConfigClap: public rclcpp::Node {
    public:
        ConfigClap();
        ~ConfigClap() override{
            serial_.close();
        }
    private:
        int param_{};
        bool pps_enable{};
        int mode;
        int width;
        int period;
        bool ins_enable = true;
        int ins_timeout{};
        float ins_align_velocity{};
        bool send_gprmc = false;
        float msg_period{};

        void try_serial_connection(std::basic_string<char> port, unsigned int baud);
        std::string port_;
        int baudrate_;
        int current_port_;
        bool different_baudrate_{false};
        int new_baudrate_;
        CallbackAsyncSerial serial_;
        std::vector<std::string> commands_;
        bool command_detected_{false};
        void serial_read_callback(const char *data, size_t len);
        std::string receive_string_;
        bool config_finish{false};
        bool config_done{false};
        bool config_save{false};
        void load_commands();
        void load_log_commands(const std::string& port, float period, std::string command);
    };
}
#endif //BUILD_CONFIG_CLAP_B7_H
