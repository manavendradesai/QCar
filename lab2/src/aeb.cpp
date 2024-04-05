// Implement automatic emergency braking using the instantaneous time to collision threshold

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

        // Declare variables
        std::vector<float> laserrange;
        bool flag_laser;

        // Constructor and name node
        AEB()
        : Node("aeb")
        {

            // Initialize variables
            // laserrange = {0};
            flag_laser = false;

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare subscriber for laserscan_msgs
            subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",default_qos,std::bind(&AEB::laserscan_callback, this,_1));

            // Declare subscriber for odom_msgs
            subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom",default_qos,std::bind(&AEB::aeb_callback, this,_1));

            // Declare publisher for ackermann_msgs
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive",10);

        }

    private:

        // Callback for laserscan message
        void laserscan_callback(const
        sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
          // Receive scan
          laserrange = msg->ranges;

          // Filter laser scan for nans and inf

          std::replace(laserrange.begin(),laserrange.end(),
          std::numeric_limits<float>::infinity(),
          msg->range_max);
          
          std::replace(laserrange.begin(),laserrange.end(),
          std::numeric_limits<float>::quiet_NaN(),
          msg->range_max);

          // // Print
          // RCLCPP_INFO(this->get_logger(), "Laser ranges: %f", laserrange[0]);

          // Set flag to true
          flag_laser = true;

        }

        // Publish aeb commands
        void aeb_callback(const
        nav_msgs::msg::Odometry::SharedPtr msg)
        {
          
          // Collect vx
          double vx = msg->twist.twist.linear.x;

          // Print
          RCLCPP_INFO(this->get_logger(), "vx: %f", vx);

          // // Execute only if filtered laser scan is received
          // if (flag_laser){

          // // Calculate instantaneous time-to-collision


          // // Determine if AEB is necessary
          // }

        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEB>());
  rclcpp::shutdown();
  return 0;
}