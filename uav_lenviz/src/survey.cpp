// This node class receives the Bi-A* time-optimal path from MissionPlanning class.

// This class will report possible collisions with other drones and calculate flight time for the Bi-A* time-optimal path.

#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

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
        float flight_time;
        int drone_id;

        // Constructor 
        Survey()
        : Node("survey")
        {

            // Initialize variables
            path_len = 0;
            v_max = 0.0;
            flight_time = 0.0;

            // Declare parameters with defaults
            this->declare_parameter("v_max",15.0);
            this->declare_parameter("drone_id",0);

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare subscriber for node path
            subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("node_path",default_qos,std::bind(&Survey::node_path_callback, this,_1));

            // Retrive actual parameters
            v_max = (this->get_parameter("v_max")).as_double();
            drone_id = (this->get_parameter("drone_id")).as_int();

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

          // Collect path length
          path_len = nodes[nodes.size()-1];

          // Considering maximum travel velocity
          flight_time = path_len/v_max;
          std::cout<<"Minimum flight time (in seconds) to destination for drone "<< drone_id <<" is: "<<flight_time<<"\n";

        }

        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Survey>());
  rclcpp::shutdown();
  return 0;
}