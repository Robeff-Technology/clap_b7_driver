//
// Created by elaydin on 25.07.2023.
//

#include "clap_b7_driver/config_clap_b7.h"
#include <fcntl.h>

namespace clap_b7{

   ConfigClap::ConfigClap() : Node("clap_b7_config"){

        port_ = this->declare_parameter<std::string>("serial_config.port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("serial_config.baudrate", 460800);
        current_port_ = this->declare_parameter<int>("serial_config.clap_port", 1);
        new_baudrate_ = baudrate_;
        load_commands();
        if(try_serial_connection(port_, baudrate_) == -1){
            exit(1);
        }
        if(!commands_.empty()){
            for(const auto& elements : commands_) {
                write_to_serial(elements.c_str(), elements.size(), true);
                if (elements.find("config com" + std::to_string(current_port_)) != std::string::npos) {
                    if (different_baudrate_) {
                        RCLCPP_INFO(this->get_logger(), "Baudrate changed to %d", new_baudrate_);
                        close(file_descriptor_);
                        try_serial_connection(port_, new_baudrate_);
                        write_to_serial(elements.c_str(), elements.size(), true);
                    }
                }
                if (elements.find("config com" + std::to_string(current_port_)) != std::string::npos) {
                    rclcpp::sleep_for(std::chrono::milliseconds(1000));
                }
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "No commands are loaded");
            rclcpp::shutdown();
        }
        write_to_serial("saveconfig\r\n", 12, true);
        write_to_serial("unlog\r\n", 7, true);
        write_to_serial("config\r\n", 8, false);
        char buffer[256];
        int bytes_read = 0;
        std::string received_data;
        rclcpp::Time start_time = this->now();
        while((this->now() - start_time).seconds() < 5.0 && rclcpp::ok())
        {
            bytes_read = read(file_descriptor_, buffer, sizeof(buffer) - 1); // -1 to leave space for null terminator
            if (bytes_read > 0) {
                received_data += std::string(buffer, bytes_read);
            }
        }
        RCLCPP_INFO(this->get_logger(), "\033[32m***************************NEW CONFIGURATIONS***************************\n %s*************************************************************\033[0m", received_data.c_str());
        RCLCPP_INFO(this->get_logger(), "Please remove the power cable and control your serial config params");

    }



    void ConfigClap::write_to_serial(std::string data, size_t len, bool receive){
        std::string removed_newline = data.substr(0, data.size() - 2);
        RCLCPP_INFO(this->get_logger(), "Sending command: %s", removed_newline.c_str());
        int size = write(file_descriptor_, data.c_str(), len);
        if(size < 0){
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", strerror(errno));
        }
        if(receive)
        {
            rclcpp::Time start_time = this->now();
            std::string received_data{};
            while((this->now() - start_time).seconds() < 3.0)
            {
                char buffer[256];
                int bytes_read = read(file_descriptor_, buffer, sizeof(buffer) - 1); // -1 to leave space for null terminator
                if (bytes_read > 0) {
                    received_data += std::string(buffer, bytes_read);
                }
            }

            if(!received_data.empty()){
                std::string response;

                size_t startPos = 0;
                size_t endPos = 0;
                while ((startPos = received_data.find('$', startPos)) != std::string::npos) {
                    endPos = received_data.find('\r', startPos);
                    if (endPos != std::string::npos) {
                        std::string substring = received_data.substr(startPos + 1, endPos - startPos - 1);
                        response += substring;
                        startPos = endPos;
                    }
                    else{
                        break;
                    }
                }
                RCLCPP_INFO(this->get_logger(), "\033[32mresponse: %s\033[0m", response.c_str());
                tcflush(file_descriptor_, TCIOFLUSH);
            }
        }
    }

    int ConfigClap::try_serial_connection(std::basic_string<char> port, unsigned int baud) {
        RCLCPP_INFO(this->get_logger(), "Trying to connect to serial port: %s", port.c_str());
        file_descriptor_ = open(port.c_str(), O_RDWR| O_NOCTTY | O_NONBLOCK);
        int opts = fcntl(file_descriptor_, F_GETFL);
        opts = opts & (O_NONBLOCK);
        fcntl(file_descriptor_, F_SETFL, opts);


        memset(&tty_, 0, sizeof(tty_));
        if (tcgetattr(file_descriptor_, &tty_) != 0) {
            RCLCPP_INFO(this->get_logger(), "Error getting serial port attributes: %s", strerror(errno));
            close(file_descriptor_);
            return -1;
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
        tty_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| IEXTEN | ECHONL*/);
        tty_.c_oflag &= ~OPOST;

        tty_.c_cc[VMIN] = 0;  // Minimum number of characters to read
        tty_.c_cc[VTIME] = 0; // Timeout in tenths of a second

        if (tcsetattr(file_descriptor_, TCSANOW, &tty_) != 0) {
            RCLCPP_INFO(this->get_logger(), "Error getting serial port attributes: %s", strerror(errno));
            close(file_descriptor_);
            return -1;
        }
        return 1;
   }

    void ConfigClap::load_commands(){
       std::string command{"unlog\r\n"};
       commands_.push_back(command);


       param_ = this->declare_parameter<int>("port1_config.baudrate", 460800);
       command = "config com1 " + std::to_string(param_) + "\r\n";
       commands_.push_back(command);

       if(current_port_ == 1){
           if(param_ != baudrate_) {
               different_baudrate_ = true;
               new_baudrate_ = param_;
           }
       }


        param_ = this->declare_parameter<int>("port2_config.baudrate", 460800);
        command = "config com2 " + std::to_string(param_) + "\r\n";
        commands_.push_back(command);

        if(current_port_ == 2){
            if(param_ != baudrate_) {
                different_baudrate_ = true;
                new_baudrate_ = param_;
            }
        }

        param_ = this->declare_parameter<int>("port3_config.baudrate", 460800);
        command = "config com3 " + std::to_string(param_) + "\r\n";
        commands_.push_back(command);

        if(current_port_ == 3){
            if(param_ != baudrate_)
            {
                different_baudrate_ = true;
                new_baudrate_ = param_;
            }
        }


        pps_enable = this->declare_parameter<bool>("pps_config.enable",  true);
        if(pps_enable){

            mode = this->declare_parameter<int>("pps_config.mode", 0);
            std::string polarity;
            polarity = this->declare_parameter<std::string>("pps_config.polarity", "POSITIVE");
            width = this->declare_parameter<int>("pps_config.width", 500000);
            period = this->declare_parameter<int>("pps_config.period", 1000);
            if(mode == 0){
                command = "config pps enable2 gps " + polarity + " " + std::to_string(width) + " " + std::to_string(period) + " 0 0" + "\r\n";
            }
            else if(mode == 1){
                command = "config pps enable gps " + polarity + " " + std::to_string(width) + " " + std::to_string(period) + " 0 0" + "\r\n";
            }
        }
        else{
            command = "config pps disable\r\n";
        }

        commands_.push_back(command);


        ins_enable = this->declare_parameter<bool>("ins_config.enable", true);
        if(ins_enable){
            command = "config ins enable\r\n";
        }
        else{
            command = "config ins disable\r\n";
        }
        commands_.push_back(command);


        ins_timeout = this->declare_parameter<int>("ins_config.timeout", 300);
        command = "config ins timeout " + std::to_string(ins_timeout) + "\r\n";
        commands_.push_back(command);


        ins_align_velocity = this->declare_parameter<float>("ins_config.align_velocity_threshold", 0.5f);
        command = "config ins alignmentvel " + std::to_string(ins_align_velocity) + "\r\n";
        commands_.push_back(command);

        std::vector<std::string> lever_arm_master;
        std::vector<std::string> lever_arm_slave;
        std::vector<std::string> lever_arm_master_error;
        std::vector<std::string> lever_arm_slave_error;
        std::vector<std::string> imu_pos_offset;
        lever_arm_master = this->declare_parameter<std::vector<std::string>>("ins_config.lever_arm_master",lever_arm_master);
        lever_arm_master_error = this->declare_parameter<std::vector<std::string>>("ins_config.lever_arm_master_error", lever_arm_master_error);

        boost::split(lever_arm_master, lever_arm_master[0], boost::is_any_of(" "));
        boost::split(lever_arm_master_error, lever_arm_master_error[0], boost::is_any_of(" "));
        command = "config imutoant offset " + lever_arm_master[0] + " " + lever_arm_master[1] + " " + lever_arm_master[2] + " " + lever_arm_master_error[0] + " " + lever_arm_master_error[1] + " " + lever_arm_master_error[2] + "\r\n";
        commands_.push_back(command);

        lever_arm_slave = this->declare_parameter<std::vector<std::string>>("ins_config.lever_arm_slave", lever_arm_slave);
        lever_arm_slave_error = this->declare_parameter<std::vector<std::string>>("ins_config.lever_arm_slave_error", lever_arm_slave_error);

        boost::split(lever_arm_slave, lever_arm_slave[0], boost::is_any_of(" "));
        boost::split(lever_arm_slave_error, lever_arm_slave_error[0], boost::is_any_of(" "));
        command = "config imutoant2 offset " + lever_arm_slave[0] + " " + lever_arm_slave[1] + " " + lever_arm_slave[2] + " " + lever_arm_slave_error[0] + " " + lever_arm_slave_error[1] + " " + lever_arm_slave_error[2] + "\r\n";
        commands_.push_back(command);

        imu_pos_offset = this->declare_parameter<std::vector<std::string>>("ins_config.imu_position_offset", imu_pos_offset);
        boost::split(imu_pos_offset, imu_pos_offset[0], boost::is_any_of(" "));
        command = "config inssol offset " + imu_pos_offset[0] + " " + imu_pos_offset[1] + " " + imu_pos_offset[2] + "\r\n";
        commands_.push_back(command);

        command = "config ododirvcc high\r\n";
        commands_.push_back(command);

        ///////////////////*********************** COM1 ***********************//////////////////////

        send_gprmc = this->declare_parameter<bool>("port1_config.gprmc", false);
        if(send_gprmc){
            command = "log com1 gprmc ontime 1\r\n";
            commands_.push_back(command);
        }

        msg_period = this->declare_parameter<float>("port1_config.uniheading_period",0.2);
        load_log_commands("com1", msg_period, "headingb");

        msg_period = this->declare_parameter<float>("port1_config.bestgnsspos_period", 0.2);
        load_log_commands("com1", msg_period, "bestgnssposb");

        msg_period = this->declare_parameter<float>("port1_config.bestgnssvel_period",0.2);
        load_log_commands("com1", msg_period, "bestgnssvelb");

        msg_period = this->declare_parameter<float>("port1_config.ecef_period",0.2);
        load_log_commands("com1", msg_period, "bestxyzb");

        msg_period = this->declare_parameter<float>("port1_config.wheel_speed_period",0.1);
        load_log_commands("com1", msg_period, "timedwheeldata");

        msg_period = this->declare_parameter<float>("port1_config.rawimu_period",0.02);
        load_log_commands("com1", msg_period, "rawimub");

        msg_period = this->declare_parameter<float>("port1_config.inspvax_period", 0.02);
        load_log_commands("com1", msg_period, "inspvaxb");




       ////////////****************COM2*******************/////////////////////
        send_gprmc = false;
        send_gprmc = this->declare_parameter<bool>("port2_config.gprmc",  false);
        if(send_gprmc){
            command = "log com2 gprmc ontime 1\r\n";
            commands_.push_back(command);
        }

        msg_period = this->declare_parameter<float>("port2_config.uniheading_period", 0.0);
        load_log_commands("com2", msg_period, "headingb");

        msg_period = this->declare_parameter<float>("port2_config.bestgnsspos_period", 0.0);
        load_log_commands("com2", msg_period, "bestgnssposb");

        msg_period = this->declare_parameter<float>("port2_config.bestgnssvel_period", 0.0);
        load_log_commands("com2", msg_period, "bestgnssvelb");

        msg_period = this->declare_parameter<float>("port2_config.ecef_period", 0.0);
        load_log_commands("com2", msg_period, "bestxyzb");

        msg_period = this->declare_parameter<float>("port2_config.wheel_speed_period", 0.0);
        load_log_commands("com2", msg_period, "timedwheeldata");

        msg_period = this->declare_parameter<float>("port2_config.rawimu_period", 0.0);
        load_log_commands("com2", msg_period, "rawimub");

        msg_period = this->declare_parameter<float>("port2_config.inspvax_period", 0.0);
        load_log_commands("com2", msg_period, "inspvaxb");


        //////******************************COM3******************************//////
        msg_period = this->declare_parameter<float>("port3_config.uniheading_period", 0.0);
        load_log_commands("com3", msg_period, "headingb");

        msg_period = this->declare_parameter<float>("port3_config.bestgnsspos_period",0.0);
        load_log_commands("com3", msg_period, "bestgnssposb");

        msg_period = this->declare_parameter<float>("port3_config.bestgnssvel_period", 0.0);
        load_log_commands("com3", msg_period, "bestgnssvelb");

        msg_period = this->declare_parameter<float>("port3_config.ecef_period", 0.0);
        load_log_commands("com3", msg_period, "bestxyzb");

        msg_period = this->declare_parameter<float>("port3_config.wheel_speed_period",0.0);
        load_log_commands("com3", msg_period, "timedwheeldata");

        send_gprmc = this->declare_parameter<bool>("port3_config.gprmc", true);
        if(send_gprmc){
            command = "log com3 gprmc ontime 1\r\n";
            commands_.push_back(command);
        }

        msg_period = this->declare_parameter<float>("port3_config.rawimu_period", 0.0);
        load_log_commands("com3", msg_period, "rawimub");

        msg_period = this->declare_parameter<float>("port3_config.inspvax_period", 0.0);
        load_log_commands("com3", msg_period, "inspvaxb");

   }

   void ConfigClap::load_log_commands(const std::string& port, float period, std::string command){
       if(period > 0.0){
              command = "log " + port + " " + command + " ontime " + std::to_string(period) + "\r\n";
              commands_.push_back(command);
       }
   }
} // namespace clap_b7

