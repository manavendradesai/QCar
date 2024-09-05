// This node class receives the Bi-A* time-optimal path from MissionPlanning class.

// This class will report possible collisions with other drones and calculate flight time for the Bi-A* time-optimal path.

// This class also simulates the position of the drone.

#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;

// Node class
class Survey : public rclcpp::Node
{

    public:

        // Declare variables
        std::vector<std::pair<float,float>> path;
        std::vector<int> nodes;

        int path_len;
        float v_max;
        float v;
        float flight_time;
        int drone_id;
        double dt;

        // Constructor 
        Survey()
        : Node("survey")
        {

            // Initialize variables
            path_len = 0;
            v_max = 0.0;
            v = 0.0;
            flight_time = 0.0;
            drone_id = 0;
            dt = 0.0;
            nodes = {0};

            // Declare parameters with defaults
            this->declare_parameter("v_max",15.0);
            this->declare_parameter("v",5.0);
            this->declare_parameter("drone_id",0);
            this->declare_parameter("dt",0.5);

            // Retrieve actual parameters
            v_max = (this->get_parameter("v_max")).as_double();
            v = (this->get_parameter("v")).as_double();
            dt = (this->get_parameter("dt")).as_double();
            drone_id = (this->get_parameter("drone_id")).as_int();

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare subscriber for node path
            subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("node_path",default_qos,std::bind(&Survey::node_path_callback, this,_1));

            // Declare publisher for drone position
            publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("drone_position",10);

        }

    private:

        // Callback to collect nodepath
        void node_path_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
        {

          // Collect message for path and path length
          nodes = msg->data;
          get_node_path(nodes);
                
        }


        // Extract node path
        void get_node_path(std::vector<int> nodes)
        {

          // Clear old path from previous iteration
          path.clear();

          // Unpack into vector of pair of node coordinates
          for(int i=1;i<=int(nodes.size()/2);i++)
          {
            path.push_back({nodes[2*i-2], nodes[2*i-1]});
          }

          // Calculate flight time
          calc_path_time();   
        }


        // Calculate shortest time to destination
        void calc_path_time()
        {

          // Collect path length. Stored in last element of the node path array
          path_len = nodes[nodes.size()-3];

          // Considering maximum travel velocity
          flight_time = path_len/v_max;
          std::cout<<"Minimum flight time (in seconds) to destination for drone "<< drone_id <<" is: "<<flight_time<<"\n";

          // Simulate drone motion
          drone_fly();
            
        }


        // Simulate drone motion
        void drone_fly()
        {

          // Lower level control commands

          // Retrieve current position of the drone
          double x0 = nodes[nodes.size()-2];
          double y0 = nodes[nodes.size()-1];

          // Head toward next node in the solution path
          double xh = nodes[4] - x0 + 0.01;
          double yh = nodes[5] - y0 + 0.01;

          // Normalized heading vector
          xh = xh/(sqrt(xh*xh + yh*yh) + 0.01);
          yh = yh/(sqrt(xh*xh + yh*yh) + 0.01);

          // Euler update for position using parameterized and constant drone speed 'v'
          x0 = x0 + v*dt*xh;
          y0 = y0 + v*dt*yh;

          // Publish updated drone position
          auto msg = geometry_msgs::msg::Pose();
          msg.position.x = x0;
          msg.position.y = y0;
          publisher_->publish(msg);
      
        }

        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_p;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;

};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Survey>());
  rclcpp::shutdown();
  return 0;
}