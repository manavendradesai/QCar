// Implement automatic emergency braking using the instantaneous time to collision threshold

// Subscribe to AckermannDriveStamped
// Subscribe to LaserScan

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

// Node class
class AEB : public rclcpp::Node
{
    public:
        // Constructor and name node
        AEB()
        : Node("aeb")
        {

            // Declare subscriber for laserscan_msgs
            subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&AEB::laserscan_callback, this,_1));

            // Declare subscriber for odom_msgs
            subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom",10,std::bind(&AEB::aeb_callback, this,_1));

            // Declare publisher for ackermann_msgs
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive",10);

        }

    private:
        void laserscan_callback(const
        sensor_msgs::msg::LaserScan::SharedPtr msg) const
        {
          // Receive scan
          float range[] = msg->ranges;
          // Filter laser scan for nans and inf
        }

        void aeb_callback(const
        nav_msgs::msg::Odometry::SharedPtr msg) const
        {
          
          // Collect vx
          double vx = msg->twist.twist.linear.x;

          // Calculate instantaneous time-to-collision

          // Determine if AEB is necessary
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEB>());
  rclcpp::shutdown();
  return 0;
}