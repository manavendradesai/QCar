#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

// Node class
class Relay : public rclcpp::Node
{
    public:
        // Constructor and name node
        Relay()
        : Node("relay")
        {

            // Declare Subscriber
            subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive",10,std::bind(&Relay::relay_callback, this,_1));

            // Declare publisher
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay",10);

        } 

    private:
        void relay_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const
        {
            
            // Receive message
            double d = msg->drive.steering_angle;
            double v = msg->drive.speed;

            d = 3*d;
            v = 3*v;

            // Define new message
            auto message = ackermann_msgs::msg::AckermannDriveStamped();
            message.drive.steering_angle = d;
            message.drive.speed = v;

            // Publish
            RCLCPP_INFO(this->get_logger(), "Publishing multiplied steering angle and speed");

            publisher_->publish(message);

        }
        
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}