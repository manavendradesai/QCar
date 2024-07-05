// Implement automatic emergency braking using the instantaneous time to collision threshold

#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2/exceptions.h"

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
        double TTC_cr;
        double laserangle;
        double xcc;
        double ycc;
        std::string fromFrameRel = "ego_racecar/laser_model";
        std::string toFrameRel = "car_center";
        // geometry_msgs::msg::PointStamped point_ray;

        // Constructor and name node
        AEB()
        : Node("aeb")
        {

            // Initialize variables
            flag_laser = false;

            //Declare parameters with defaults
            this->declare_parameter("TTC_cr",2.0);

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare subscriber for laserscan_msgs
            subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",default_qos,std::bind(&AEB::laserscan_callback, this,_1));

            // Declare subscriber for odom_msgs
            subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom",default_qos,std::bind(&AEB::aeb_callback, this,_1));

            // Declare publisher for ackermann_msgs
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive",10);

            // Transform listerners
            // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

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

          // Set flag to true
          flag_laser = true;

        }

        // Publish aeb commands
        void aeb_callback(const
        nav_msgs::msg::Odometry::SharedPtr msg)
        {
          
          // Collect vx
          double vx = msg->twist.twist.linear.x;

          // // Publish
          // RCLCPP_INFO(this->get_logger(), "vx: %f",vx);
      
          // Collect critical time-to-collision-threshold
          TTC_cr = (this->get_parameter("TTC_cr")).as_double();

          // Execute only if filtered laser scan is received
          if (flag_laser){

            float min_iTTc = 100;
            
            // Calculate instantaneous time-to-collision
            for (unsigned int i=0;i<laserrange.size();i++){

              // Transform ranges to car_center frame
              laserangle = laserangle_min+i*laserangle_inc;
              
              // // Coordinates in laser_model frame
              // xcc = laserrange[i]*cos(laserangle);
              // ycc = laserrange[i]*sin(laserangle);

              // // Create Point Stamped message
              // point_ray.header.stamp = this->get_clock()->now();
              // point_ray.header.frame_id = fromFrameRel;
              // point_ray.point.x = xcc;
              // point_ray.point.y = ycc;
              // point_ray.point.z = 0.0;

              // // Retrieve transforms
              // try {
              //   // tf_buffer_->transform(point_ray, point_ray, toFrameRel);


              //   RCLCPP_INFO(
              //     this->get_logger(), "Point laser ray in frame of car_center: x:%f y:%f z:%f\n",
              //     point_ray.point.x,
              //     point_ray.point.y,
              //     point_ray.point.z);
              // } catch (const tf2::TransformException & ex) {
              //   RCLCPP_WARN(
              //     // Print exception which was caught
              //     this->get_logger(), "Unable to transform %s\n", ex.what());
              //     }

              // // Retrieve points from PointStampedMessage
              // xcc = point_ray.point.x;
              // ycc = point_ray.point.y;

              // // Revise range and polar angle in car_center frame
              // laserrange[i] = sqrt(xcc*xcc + ycc*ycc);
              // laserangle = atan2(ycc,xcc); 
              
              float iTTc = laserrange[i]/(std::max(0.001,vx*cos(laserangle)));

              min_iTTc = std::min(min_iTTc,iTTc);      
            
            }

            // // Publish
            // RCLCPP_INFO(this->get_logger(), "min_iTTc: %f",min_iTTc);

            // Determine if AEB is necessary
            if (min_iTTc<TTC_cr){

              // Publish brakes
              auto message = ackermann_msgs::msg::AckermannDriveStamped();
              message.drive.speed = 0.0;

              // Publish
              RCLCPP_INFO(this->get_logger(), "AEB applied!!");
              publisher_->publish(message);

            }

            // Reset flag
            flag_laser = false;

          }

        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

        // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEB>());
  rclcpp::shutdown();
  return 0;
}