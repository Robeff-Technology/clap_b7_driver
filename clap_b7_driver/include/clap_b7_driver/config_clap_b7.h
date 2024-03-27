//
// Created by elaydin on 25.07.2023.
//

#ifndef CONFIG_CLAP_B7_H
#define CONFIG_CLAP_B7_H


#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <boost/format.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
namespace clap_b7{

    class ConfigClap: public rclcpp::Node {
    public:
        ConfigClap();
        ~ConfigClap(){
            close(file_descriptor_);
        };
        void Update();
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
        int file_descriptor_;

        struct termios tty_;
        int try_serial_connection(std::basic_string<char> port, unsigned int baud);
        std::string port_;
        int baudrate_;
        int current_port_;
        bool different_baudrate_{false};
        int new_baudrate_;
        std::vector<std::string> commands_;
        void load_commands();
        void load_log_commands(const std::string& port, float period, std::string command);
        void write_to_serial(std::string data, size_t len, bool receive);
    };
}
#endif //BUILD_CONFIG_CLAP_B7_H
