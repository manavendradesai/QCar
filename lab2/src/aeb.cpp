// Implement automatic emergency braking using the instantaneous time to collision threshold

#include <memory>
#include <cmath>

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
        float laserangle_min;
        float laserangle_inc;
        bool flag_laser;

        // Constructor and name node
        AEB()
        : Node("aeb")
        {

            // Initialize variables
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
          laserangle_min = msg->angle_min;
          laserangle_inc = msg->angle_increment;

          // Filter laser scan for inf
          std::replace(laserrange.begin(),laserrange.end(),
          std::numeric_limits<float>::infinity(),
          msg->range_max);

          // Filter laser scan for nans
          std::replace_if(laserrange.begin(), laserrange.end(), [](float x) { return std::isnan(x); }, msg->range_max); 

          // // Print
          // RCLCPP_INFO(this->get_logger(), "Laserangle min : %f", laserangle_min);

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


          // Execute only if filtered laser scan is received
          if (flag_laser){

            float min_iTTc = 100;
            
            // Calculate instantaneous time-to-collision
            for (unsigned int i=0;i<laserrange.size();i++){
              
              float iTTc = laserrange[i]/(std::max(0.001,vx*cos(laserangle_min+i*laserangle_inc)));

              min_iTTc = std::min(min_iTTc,iTTc);      
            
            }

            // Determine if AEB is necessary
            if (min_iTTc<1){

              // Publish brakes
              auto message = ackermann_msgs::msg::AckermannDriveStamped();
              message.drive.speed = 0;

              // Publish
              RCLCPP_INFO(this->get_logger(), "Applying AEB!!");
              publisher_->publish(message);

            }

          }

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