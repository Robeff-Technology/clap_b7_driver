//
// Created by elaydin on 25.07.2023.
//

#include "clap_b7_driver/config_clap_b7.h"
namespace clap_b7{

   ConfigClap::ConfigClap() : Node("clap_b7_config"){
        success_cmd_cnt_ = 1;
        rclcpp::NodeOptions node_opt;
        node_opt.automatically_declare_parameters_from_overrides(true);
        rclcpp::Node n_private("npv", "", node_opt);
        n_private.get_parameter_or<std::string>("serial_config.port", port_, "/dev/ttyUSB0");
        n_private.get_parameter_or<int>("serial_config.baudrate", baudrate_, 460800);
        n_private.get_parameter_or<int>("serial_config.clap_port", current_port, 1);
        load_commands(n_private);
        if(different_baudrate){
            RCLCPP_INFO(this->get_logger(), "Different baudrate is set for port %d current baudrate = %d new baudrate = %d", current_port, baudrate_, new_baudrate_);
        }
        ConfigClap::try_serial_connection(port_, baudrate_);
        serial_.setCallback(std::bind(&ConfigClap::serial_read_callback, this, std::placeholders::_1, std::placeholders::_2));
        if(!commands_.empty()){
            for(const auto& elements : commands_){
                RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");
                RCLCPP_INFO(this->get_logger(), "Sending command: %s", elements.c_str());
                serial_.write(elements.c_str(), elements.size());

                if(elements.find("config com" + std::to_string(current_port)) != std::string::npos){
                    if(different_baudrate){
                    RCLCPP_INFO(this->get_logger(), "Changing baudrate to %d", new_baudrate_);
                    try_serial_connection(port_, new_baudrate_);
                    success_cmd_cnt_++;
                    }
                }
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "No commands are loaded");
            rclcpp::shutdown();
        }

        if(success_cmd_cnt_ == commands_.size()){
            RCLCPP_INFO(this->get_logger(), "\033[32mAll commands are sent successfully\033[0m");
            serial_.write("saveconfig\r\n", 12);
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Some commands are not sent successfully");
        }
       rclcpp::shutdown();
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
        }while(!serial_.isOpen());

        RCLCPP_INFO(this->get_logger(), "\033[32mConnected to serial port(%s, %s)\033[0m", port.c_str(), std::to_string(baud).c_str());
    }

    void ConfigClap::serial_read_callback(const char *data, size_t len) {

        for(size_t i = 0; i < len; i++){
            if(data[i] == '$'){
                command_detected_ = true;
            }
            if(command_detected_){
                if(data[i] == '\n'){
                    command_detected_ = false;
                    if(receive_string_.find("OK") != std::string::npos){
                        success_cmd_cnt_++;
                        RCLCPP_INFO(this->get_logger(), "\033[32mCommand loaded successfully: %s\033[0m", receive_string_.c_str());
                    }
                    receive_string_.clear();
                }
                else{
                    receive_string_ += data[i];
                }

            }
        }
    }

    void ConfigClap::load_commands(const rclcpp::Node& node){

       std::string command{"unlog\r\n"};
       commands_.push_back(command);

       int param_{};
       node.get_parameter_or<int>("port1_config.baudrate", param_, 460800);
       command = "config com1 " + std::to_string(param_) + "\r\n";
       commands_.push_back(command);

       if(current_port == 1){
           if(param_ != baudrate_) {
               different_baudrate = true;
               new_baudrate_ = param_;
           }
       }


        node.get_parameter_or<int>("port2_config.baudrate", param_, 460800);
        command = "config com2 " + std::to_string(param_) + "\r\n";
        commands_.push_back(command);

        if(current_port == 2){
            if(param_ != baudrate_) {
                different_baudrate = true;
                new_baudrate_ = param_;
            }
        }

        node.get_parameter_or<int>("port3_config.baudrate", param_, 460800);
        command = "config com3 " + std::to_string(param_) + "\r\n";
        commands_.push_back(command);

        if(current_port == 3){
            if(param_ != baudrate_)
            {
                different_baudrate = true;
                new_baudrate_ = param_;
            }
        }

        bool pps_enable;
        node.get_parameter_or<bool>("pps_config.enable", pps_enable, true);
        if(pps_enable){
            int mode;
            node.get_parameter_or<int>("pps_config.mode", mode, 0);
            std::string polarity;
            node.get_parameter_or<std::string>("pps_config.polarity", polarity, "POSITIVE");
            int width;
            node.get_parameter_or<int>("pps_config.width", width, 500000);
            int period;
            node.get_parameter_or<int>("pps_config.period", period, 1000);
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

        bool ins_enable = true;
        node.get_parameter_or<bool>("ins_config.enable", ins_enable, true);
        if(ins_enable){
            command = "config ins enable\r\n";
        }
        else{
            command = "config ins disable\r\n";
        }
        commands_.push_back(command);

        int ins_timeout{};
        node.get_parameter_or<int>("ins_config.timeout", ins_timeout, 300);
        command = "config ins timeout " + std::to_string(ins_timeout) + "\r\n";
        commands_.push_back(command);

        float ins_align_velocity{};
        node.get_parameter_or<float>("ins_config.align_velocity_threshold", ins_align_velocity, 0.5);
        command = "config ins alignmentvel " + std::to_string(ins_align_velocity) + "\r\n";
        commands_.push_back(command);

        std::vector<std::string> lever_arm_master;
        std::vector<std::string> lever_arm_slave;
        std::vector<std::string> lever_arm_master_error;
        std::vector<std::string> lever_arm_slave_error;

        node.get_parameter_or<std::vector<std::string>>("ins_config.lever_arm_master", lever_arm_master, lever_arm_master);
        node.get_parameter_or<std::vector<std::string>>("ins_config.lever_arm_master_error", lever_arm_master_error, lever_arm_master_error);

        boost::split(lever_arm_master, lever_arm_master[0], boost::is_any_of(" "));
        boost::split(lever_arm_master_error, lever_arm_master_error[0], boost::is_any_of(" "));
        command = "config imutoant offset " + lever_arm_master[0] + " " + lever_arm_master[1] + " " + lever_arm_master[2] + " " + lever_arm_master_error[0] + " " + lever_arm_master_error[1] + " " + lever_arm_master_error[2] + "\r\n";
        commands_.push_back(command);

        node.get_parameter_or<std::vector<std::string>>("ins_config.lever_arm_slave", lever_arm_slave, lever_arm_slave);
        node.get_parameter_or<std::vector<std::string>>("ins_config.lever_arm_slave_error", lever_arm_slave_error, lever_arm_slave_error);

        boost::split(lever_arm_slave, lever_arm_slave[0], boost::is_any_of(" "));
        boost::split(lever_arm_slave_error, lever_arm_slave_error[0], boost::is_any_of(" "));
        command = "config imutoant2 offset " + lever_arm_slave[0] + " " + lever_arm_slave[1] + " " + lever_arm_slave[2] + " " + lever_arm_slave_error[0] + " " + lever_arm_slave_error[1] + " " + lever_arm_slave_error[2] + "\r\n";
        commands_.push_back(command);

        ///////////////////*********************** COM1 ***********************//////////////////////
        bool send_gprmc = false;
        node.get_parameter_or<bool>("port1_config.gprmc", send_gprmc, false);
        if(send_gprmc){
            command = "log com1 gprmc ontime 1\r\n";
            commands_.push_back(command);
        }
       float period{};
        node.get_parameter_or<float>("port1_config.uniheading_period", period, 0.2);
        load_log_commands("com1", period, "headingb");

        node.get_parameter_or<float>("port1_config.bestgnsspos_period", period, 0.2);
        load_log_commands("com1", period, "bestgnssposb");

        node.get_parameter_or<float>("port1_config.bestgnssvel_period", period, 0.2);
        load_log_commands("com1", period, "bestgnssvelb");

        node.get_parameter_or<float>("port1_config.ecef_period", period, 0.2);
        load_log_commands("com1", period, "bestxyzb");

       node.get_parameter_or<float>("port1_config.rawimu_period", period, 0.05);
       load_log_commands("com1", period, "rawimub");

        node.get_parameter_or<float>("port1_config.inspvax_period", period, 0.02);
       load_log_commands("com1", period, "inspvaxb");


       ////////////****************COM2*******************/////////////////////
        send_gprmc = false;
        node.get_parameter_or<bool>("port2_config.gprmc", send_gprmc, false);
        if(send_gprmc){
            command = "log com2 gprmc ontime 1\r\n";
            commands_.push_back(command);
        }

        node.get_parameter_or<float>("port2_config.uniheading_period", period, 0.0);
        load_log_commands("com2", period, "headingb");

        node.get_parameter_or<float>("port2_config.bestgnsspos_period", period, 0.0);
        load_log_commands("com2", period, "bestgnssposb");

        node.get_parameter_or<float>("port2_config.bestgnssvel_period", period, 0.0);
        load_log_commands("com2", period, "bestgnssvelb");

        node.get_parameter_or<float>("port2_config.ecef_period", period, 0.0);
        load_log_commands("com2", period, "bestxyzb");

        node.get_parameter_or<float>("port2_config.rawimu_period", period, 0.0);
        load_log_commands("com2", period, "rawimub");

        node.get_parameter_or<float>("port2_config.inspvax_period", period, 0.0);
        load_log_commands("com2", period, "inspvaxb");


        //////******************************COM3******************************//////
        node.get_parameter_or<float>("port3_config.uniheading_period", period, 0.0);
        load_log_commands("com3", period, "headingb");

        node.get_parameter_or<float>("port3_config.bestgnsspos_period", period, 0.0);
        load_log_commands("com3", period, "bestgnssposb");

        node.get_parameter_or<float>("port3_config.bestgnssvel_period", period, 0.0);
        load_log_commands("com3", period, "bestgnssvelb");

        node.get_parameter_or<float>("port3_config.ecef_period", period, 0.0);
        load_log_commands("com3", period, "bestxyzb");

        send_gprmc = false;
        node.get_parameter_or<bool>("port3_config.gprmc", send_gprmc, true);
        if(send_gprmc){
            command = "log com3 gprmc ontime 1\r\n";
            commands_.push_back(command);
        }

        node.get_parameter_or<float>("port3_config.rawimu_period", period, 0.0);
        load_log_commands("com3", period, "rawimub");

        node.get_parameter_or<float>("port3_config.inspvax_period", period, 0.0);
        load_log_commands("com3", period, "inspvaxb");

   }

   void ConfigClap::load_log_commands(const std::string& port, float period, std::string command){
       if(period > 0.0){
              command = "log " + port + " " + command + " ontime " + std::to_string(period) + "\r\n";
              commands_.push_back(command);
       }
   }



} // namespace clap_b7

