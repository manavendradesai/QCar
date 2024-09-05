// This node class muxes positions of all drones into a single array.

#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;

// Node class
class MuxDronePos : public rclcpp::Node
{

    public:

        // Declare variables
        double drone_pos[4];
        bool flag_1, flag_2;

        // Constructor 
        MuxDronePos()
        : Node("mux_drone_pos")
        {

            // Initialize variables
            memset(drone_pos, 0.0, sizeof(drone_pos));
            flag_1 = false;
            flag_2 = false;

            // Declare parameters with defaults

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare subscriber for drone positions
            subscription_1 = this->create_subscription<geometry_msgs::msg::Pose>("/uav_1/drone_position",default_qos,std::bind(&MuxDronePos::drone_pos_1, this,_1));

            subscription_2 = this->create_subscription<geometry_msgs::msg::Pose>("/uav_2/drone_position",default_qos,std::bind(&MuxDronePos::drone_pos_2, this,_1));

            // Declare publisher for array of drone positions
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/all_drone_positions",10);


        }

    private:

        // Callbacks
        void drone_pos_1(const geometry_msgs::msg::Pose::SharedPtr msg)
        {

            // Collect drone position
            drone_pos[0] = msg->position.x;
            drone_pos[1] = msg->position.y;

            flag_1 = true;
            publish_all_pos();

              
        }


        void drone_pos_2(const geometry_msgs::msg::Pose::SharedPtr msg)
        {

            // Collect drone position
            drone_pos[2] = msg->position.x;
            drone_pos[3] = msg->position.y;

            flag_2 = true;
            publish_all_pos();

        }


        // Mux positions and publish
        void publish_all_pos()
        {

            // Execute only once positions of all drones are received
            if (flag_1 && flag_2)
            {

                auto msg = std_msgs::msg::Float64MultiArray();

                // Convert float array to vector
                std::vector<double> v(drone_pos, drone_pos + sizeof drone_pos / sizeof drone_pos[0]);

                msg.data = v;
                publisher_->publish(msg);

                // Reset flags
                flag_1 = false;
                flag_2 = false;

            }
        }

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_1;

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_2;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MuxDronePos>());
  rclcpp::shutdown();
  return 0;
}