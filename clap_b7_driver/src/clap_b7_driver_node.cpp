#include "clap_b7_driver/clap_b7_driver.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<clap_b7::ClapB7Driver>(rclcpp::NodeOptions());
    rclcpp::Rate rate(200); // it has to be update minimum 200Hz
    while(rclcpp::ok()){
        node->Update();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}


