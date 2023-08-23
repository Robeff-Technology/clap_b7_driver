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
    private:
        void try_serial_connection(std::basic_string<char> port, unsigned int baud);
        std::string port_;
        int baudrate_;
        int current_port;
        bool different_baudrate;
        int new_baudrate_;
        CallbackAsyncSerial serial_;
        std::vector<std::string> commands_;
        bool command_detected_{false};
        void serial_read_callback(const char *data, size_t len);
        std::string receive_string_;
        int success_cmd_cnt_;
        void load_commands(const rclcpp::Node& node);
        void load_log_commands(const std::string& port, float period, std::string command);
    };
}
#endif //BUILD_CONFIG_CLAP_B7_H
