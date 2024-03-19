#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

// Node class
class Talker : public rclcpp::Node
{
    public:
        // Constructor and name node
        Talker()
        : Node("talker")
        {
            
            //Declare parameters with defaults
            this->declare_parameter("d",0.0);
            this->declare_parameter("v",0.0);

            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive",10);

            timer_ = this->create_wall_timer(0.5ms, std::bind(&Talker::talker_callback, this));
             
        }

    private:
        // Function to publish
        void talker_callback()
        {

            auto message = ackermann_msgs::msg::AckermannDriveStamped();

            // Get parameters
            // Set message fields to parameters
            message.drive.steering_angle = (this->get_parameter("d")).as_double();
            message.drive.speed = (this->get_parameter("v")).as_double();

            RCLCPP_INFO(this->get_logger(), "Publishing steering angle and speed");

            publisher_->publish(message);

        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
