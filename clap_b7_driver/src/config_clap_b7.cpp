//
// Created by elaydin on 25.07.2023.
//

#include "clap_b7_driver/config_clap_b7.h"
namespace clap_b7{

   ConfigClap::ConfigClap() : Node("clap_b7_config"){

        port_ = this->declare_parameter<std::string>("serial_config.port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("serial_config.baudrate", 460800);
        current_port_ = this->declare_parameter<int>("serial_config.clap_port", 1);
        new_baudrate_ = baudrate_;
        load_commands();

        ConfigClap::try_serial_connection(port_, baudrate_);
        serial_.setCallback(std::bind(&ConfigClap::serial_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        if(!commands_.empty()){
            for(const auto& elements : commands_){
                std::string removed_newline = elements.substr(0, elements.size() - 2);
                RCLCPP_INFO(this->get_logger(), "Sending command: %s", removed_newline.c_str());
                serial_.write(elements.c_str(), elements.size());

                if(elements.find("config com" + std::to_string(current_port_)) != std::string::npos){
                        if(different_baudrate_){
                        RCLCPP_INFO(this->get_logger(), "Baudrate changed to %d", new_baudrate_);
                        try_serial_connection(port_, new_baudrate_);
                        serial_.write(elements.c_str(), elements.size()); // clap b7 has a bug when baudrate change
                    }
                }
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "No commands are loaded");
            rclcpp::shutdown();
        }
        {
            config_finish = true;
            serial_.write("saveconfig\r\n", 12);
            rclcpp::sleep_for(std::chrono::seconds(2));
            serial_.write("unlog\r\n", 7);
            rclcpp::sleep_for(std::chrono::seconds(2));
            serial_.write("config\r\n", 8);
            config_finish = false;
            config_save = true;
            rclcpp::sleep_for(std::chrono::seconds(5));
            if(config_done){
                RCLCPP_INFO(this->get_logger(), "\033[32m----------------NEW CONFIGURATIONS-------------------\n%s----------------CONFIGURATIONS SAVED-------------------\033[0m", receive_string_.c_str());
                RCLCPP_INFO(this->get_logger(), "Please remove the power cable and restart the CLAP");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "\033[31m----------------CONFIGURATIONS COULD NOT BE SAVED-------------------\033[0m");
                RCLCPP_INFO(this->get_logger(), "Please remove the power cable and control your serial config params");
            }
        }
    }

    void ConfigClap::try_serial_connection(std::basic_string<char> port, unsigned int baud) {
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
        }while(!serial_.isOpen() && rclcpp::ok());

        if(serial_.isOpen()){
            RCLCPP_INFO(this->get_logger(), "\033[32mConnected to serial port(%s, %s)\033[0m", port.c_str(), std::to_string(baud).c_str());
        }
    }

    void ConfigClap::serial_read_callback(const char *data, size_t len) {
        if(config_finish){
            for(size_t i = 0; i < len; i++) {
                if (data[i] == '$') {
                    command_detected_ = true;
                }
                if (command_detected_) {
                    if (data[i] == '\n') {
                        command_detected_ = false;
                        if (receive_string_.find("OK") != std::string::npos) {
                            RCLCPP_INFO(this->get_logger(), "\033[32mCommand loaded successfully: %s\033[0m",
                                        receive_string_.c_str());
                            config_done = true;
                            serial_.write("unlog\r\n", 12);
                        }
                        receive_string_.clear();
                    }
                    else {
                        receive_string_ += data[i];
                    }
                }
            }
        }
        if(config_save){
            receive_string_ += std::string(data, len);
        }
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


        ins_align_velocity = this->declare_parameter<float>("ins_config.align_velocity_threshold", 0.5);
        command = "config ins alignmentvel " + std::to_string(ins_align_velocity) + "\r\n";
        commands_.push_back(command);

        std::vector<std::string> lever_arm_master;
        std::vector<std::string> lever_arm_slave;
        std::vector<std::string> lever_arm_master_error;
        std::vector<std::string> lever_arm_slave_error;

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

