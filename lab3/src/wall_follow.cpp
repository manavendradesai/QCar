// Implement wall following using the instantaneous LiDAR data and PID steering

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

using std::placeholders::_1;

// Node class
class WallFollow : public rclcpp::Node
{

    public:

        // Declare variables
        std::vector<float> laserrange;
        float laserangle_min;
        float laserangle_inc;
        bool flag_laser;
        bool flag_error;

        // Zero of LiDAR is along x of base-link
        float rnghead_low;
        float rnghead_high;
        float theta;
        float L;

        // PID gains
        float kp;
        float kd;

        // Error history
        float pres_error;
        float prev_error;

        // Desired distance to the wall in meters
        float dist_to_wall;

        // Constructor and name the node
        WallFollow()
        : Node("wall_follow")
        {

            // Initialize variables
            flag_laser = false;
            flag_error = false;
            pres_error = 0;
            prev_error = 0;

            // Declare parameters with defaults
            this->declare_parameter("rnghead_low",30*(M_PI/180));
            this->declare_parameter("rnghead_high",90*(M_PI/180));
            this->declare_parameter("L",1.5);
            this->declare_parameter("kp",0.5);
            this->declare_parameter("kd",5.0);
            this->declare_parameter("dist_to_wall",0.8);

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare subscriber for laserscan_msgs
            subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",default_qos,std::bind(&WallFollow::laserscan_callback, this,_1));

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

          // Set flag to true
          flag_laser = true;

          // Get tracking error and parameters
          rnghead_high = (this->get_parameter("rnghead_high")).as_double();
          rnghead_low = (this->get_parameter("rnghead_low")).as_double();
          dist_to_wall = (this->get_parameter("dist_to_wall")).as_double();
          L = (this->get_parameter("L")).as_double();
          kp = (this->get_parameter("kp")).as_double();
          kd = (this->get_parameter("kd")).as_double();

          get_error(rnghead_high, rnghead_low);

          // Publish commands for PID steering
          if (flag_error){
            pid_control();
          }

        }


        // Helper function to get laserscan range for given angle
        double get_range(std::vector<float> range, double angle)
        {

            // Run only if laserscan is received and filtered
            if (flag_laser) {

                // Index of heading. Neglect decimal.
                int i = trunc( (angle-laserangle_min)/laserangle_inc );

                return range[i];
            }

            return 0;

        }


        // Get cross-track error from left wall
        void get_error(float rng_high, float rng_low)
        {

            // RCLCPP_INFO(this->get_logger(), "Entered get_error");

            float r_high = get_range(laserrange,rng_high);
            float r_low = get_range(laserrange,rng_low);
            float theta = r_high - r_low;

            float alpha = atan2( (r_high - r_low*cos(theta)) , (r_low*sin(theta) + 0.001) );

            float r = r_high*cos(alpha);

            pres_error = dist_to_wall - (r - L*sin(alpha));

            // Set flag to true
            flag_error = true;

            // RCLCPP_INFO(this->get_logger(), "alpha: %f",alpha);

        }

        // Set PID control
        void pid_control()
        {

        // PID steering angle (in radians)
        double pid_steer = -kp*pres_error - kd*(pres_error - prev_error);

        // Limit the steering angle to 25 degrees
        pid_steer = std::min(25*M_PI/180, pid_steer);
        pid_steer = std::max(-25*M_PI/180, pid_steer);

        // Publish PID commands for steering angle, and velocity
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.drive.steering_angle = pid_steer;

        msg.drive.speed = 1.5 - 0.5*trunc( (abs(pid_steer)*180/M_PI) / 10);

        // Publish
        RCLCPP_INFO(this->get_logger(), "Publishing speed: %f", msg.drive.speed);

        RCLCPP_INFO(this->get_logger(), "Publishing steering angle: %f", pid_steer);

        publisher_->publish(msg);

        // Update past error
        prev_error = pres_error;

        // Reset flags
        flag_error = false;
        flag_laser = false;

        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollow>());
  rclcpp::shutdown();
  return 0;
}