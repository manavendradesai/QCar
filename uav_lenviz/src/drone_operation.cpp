// This class contains methods to navigate an unmanned aerial vehicle. The navigation commands include takeoff, survey, return-to-home, land, and failure.

#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

// Node class
class DroneOperation : public rclcpp::Node
{

    public:

        // Declare variables

        // Constructor
        DroneOperation()
        : Node("drone_operation")
        {

            // Initialize variables

            // Declare parameters with default values

            // Declare subscribers

            // Declare publishers

        }


    private:

        // Callbacks

        // Take-off navigation command
        void take_off()
        {
            
            // Pass source and destination

            RCLCPP_INFO(this->get_logger(), "Taking-off...");
        }

        // Survey navigation command
        void survey()
        {
            RCLCPP_INFO(this->get_logger(), "Surveying...");
        }

        // Return-to-home navigation command
        void homing()
        {
            RCLCPP_INFO(this->get_logger(), "Returning home...");
        }
        
        // Land navigation command
        void land()
        {
            RCLCPP_INFO(this->get_logger(), "Landing...");
        }

        // Failure and graceful-degradation command
        void failure()
        {
            RCLCPP_INFO(this->get_logger(), "Failure!!!");
        }

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneOperation>());
    rclcpp::shutdown();
    return 0;
}