// This node first creates a 2D graph for use in motion planning for a drone.

// Then, this node applies Bidirectional A* to find a time optimal path from a source to a destination.

// Instantaneous positions of the drones are received and added as obstacles to the graph for planning and obstacle avoidance

// The code for Bi-A* is adapted from https://github.com/akshay-antony/BiDirectionalWeightedAstar/blob/main/src/main.cpp

#include "uav.h"

#include <memory>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

// Node class
class MissionPlanning : public rclcpp::Node
{

    public:

        // Declare variables
        std::vector<double> drone_pos;
        std::vector<std::pair<int,int>> input_obstacles;

        std::pair<float,float> xy0;
        std::pair<float,float> start_point;
        std::pair<float,float> goal_point;
        
        int drone_id;
        int drone_num;
        float crit_gap;

        std::vector<int> nodes;
        u_int32_t path_len;

        std::vector<int> drone_obs;

        // Constructor
        MissionPlanning()
        : Node("mission_plan")
        {

            // Initialize variables
            start_point = {1.0,1.0};
            goal_point = {1.0,1.0};
            drone_id = 0;
            path_len = 0;
            drone_num = 0;
            crit_gap = 0.0;

            // Declare parameters
            this->declare_parameter("start_x",1.0);
            this->declare_parameter("start_y",1.0);
            this->declare_parameter("goal_x",1.0);
            this->declare_parameter("goal_y",1.0);
            this->declare_parameter("drone_id",0);
            this->declare_parameter("drone_num",0);
            this->declare_parameter("crit_gap",0.0);

            // Retrieve parameters.
            start_point.first = (this->get_parameter("start_x")).as_double();
            start_point.second = (this->get_parameter("start_y")).as_double();
            goal_point.first = (this->get_parameter("goal_x")).as_double();
            goal_point.second = (this->get_parameter("goal_y")).as_double();
            drone_id = (this->get_parameter("drone_id")).as_int();
            drone_num = (this->get_parameter("drone_num")).as_int();
            crit_gap = (this->get_parameter("crit_gap")).as_double();

            // Prepare vector of ids of drones to be treated as obstacles
            std::vector<int> num_drone(drone_num);
            iota(num_drone.begin(), num_drone.end(), 1);
            num_drone.erase(num_drone.begin() + drone_id - 1);
            drone_obs = num_drone;

            xy0 = start_point;   

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            // Declare publisher for node path
            publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("node_path",10);

            // Subscribe to drone positions
            subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/all_drone_positions",default_qos,std::bind(&MissionPlanning::set_drone_pos, this,_1));

            // Trigger planner
            plan_mission();

        }


    private:

        // Callback to collect drone position and obstacle positions
        void set_drone_pos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {

            drone_pos = msg->data;
            split_pos(drone_pos);
            
        }

        // Segregate drone and obstacle positions
        void split_pos(std::vector<double> drone_pos)
        {

            // Drone position
            xy0.first = drone_pos[2*drone_id-2];
            xy0.second = drone_pos[2*drone_id-1];

            // Real-time positions of other drones can be treated as occupied cells. Retrieve positions of other drones to add as obstacles. Convert to node. 
            for (auto x: drone_obs)
            {
                input_obstacles.push_back({drone_pos[2*x-2], drone_pos[2*x-1]});
            }

            // Interdrone distance. Currently only for a two-drone task.
            float gap = sqrt(pow((drone_pos[0]-drone_pos[2]),2) + pow((drone_pos[1]-drone_pos[3]),2));
            std::cout<<"Inter drone distance..."<<gap<<"\n";

            // Check for collision
            if (gap>crit_gap)
            {
            // Plan mission path
            plan_mission();
            }
            else
            {
            std::cout<<"Collision imminent!!!"<<"\n";
            }

        }

        int plan_mission()
        {

            // Start timing solve duration
            auto start_time = std::chrono::high_resolution_clock::now();

            // Point drone position to a node
            start_point.first = ceil(xy0.first);
            start_point.second = ceil(xy0.second);

            // Add occupied cells.
            // Add static obstacle.
            input_obstacles.push_back({200, 200});
            
            // Each cell of the map contains {x_cell, y_cell, f, h, g, is_occ}
            std::map<std::pair<float,float>,std::vector<float>> all_points_fwd, all_points_bwd;

            gp::pq open_points_fwd, open_points_bwd;

            std::set<std::pair<float,float>> closed_points_fwd, closed_points_bwd;

            // Convert map to graph
            make_graph(input_obstacles, all_points_fwd);
            all_points_bwd = all_points_fwd;

            // Check if start and goal positions are within graph
            if(!is_valid(start_point.first, start_point.second) || !is_valid(goal_point.first, goal_point.second)){
                std::cout<<"\n Start or Goal Point out of bounds...";
                return 0;
            }
            // Check if start and goal positions are on an occupied cell
            else if(!is_not_obstacle(start_point.first, start_point.second, all_points_fwd) 
                    || !is_not_obstacle(goal_point.first,goal_point.second, all_points_fwd)){
                std::cout<<"\n Start or Goal Point is an obstacle...";
                return 0;
            }

            all_points_fwd[start_point] = {start_point.first, start_point.second, 0., 0., 0., 0.};

            open_points_fwd.push({all_points_fwd[start_point][2], start_point.first, start_point.second});

            all_points_bwd[goal_point] = {goal_point.first, goal_point.second, 0., 0., 0., 0.};

            open_points_bwd.push({all_points_bwd[goal_point][2], goal_point.first, goal_point.second});

            // Call Bi-A* planner on two different threads for forward and backward search, respectively
            std::thread t1(&MissionPlanning::plan, this, std::ref(all_points_fwd), std::ref(open_points_fwd), std::ref(closed_points_fwd), true);

            std::thread t2(&MissionPlanning::plan, this, std::ref(all_points_bwd), std::ref(open_points_bwd), std::ref(closed_points_bwd), false);
            
            t1.join();
            t2.join();

            // std::cout<<"Created both threads..."<<"\n";

            if(gp::found)
                trace_path(all_points_fwd, all_points_bwd);
            else
                std::cout<<"No path Found"<<std::endl;

            // Stop clock for solve-time
            auto stop_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time-start_time);
            
            std::cout<<"Path planning time for drone "<<drone_id<<" (in milliseconds): "<<duration.count()<<std::endl;

            // Reset Bi-A* variables for next planning iteration
            gp::found = false;
            gp::visited_fwd = {};
            gp::visited_bwd = {};
            nodes.clear();
            input_obstacles.clear();

            return 0;

        }


        // Check if the point is inside the map
        bool is_valid(const float x, const float y){
            if(x < gp::height && y < gp::width && x >= 0 && y >= 0)
                return true; 
            return false;
        }


        // Check if the point is on an occupied cell
        bool is_not_obstacle(const float x, const float y, std::map<std::pair<float,float>,std::vector<float>>& all_points){
            if(all_points[{x,y}][5] == 0)
                return true;
            return false;
        }


        // Check if the point is already explored
        bool is_in_closed_set(const float x, const float y, const std::set<std::pair<float,float>>& closed_points){
            bool res = (closed_points.find({x,y}) != closed_points.end())?true:false;
            return res;
        }


        // Make graph
        void make_graph(std::vector<std::pair<int,int>>& input_obstacles,
                        std::map<std::pair<float,float>,std::vector<float>>& all_points){
            for(int i=0;i<gp::height;++i){
                for(int j=0;j<gp::width;++j){
                    all_points[{i,j}] = {-1, -1, FLT_MAX, FLT_MAX, FLT_MAX,0};
                }
            }
            for(auto x: input_obstacles){
                all_points[x][5] = 1;
            }

            std::cout<<"Made graph for "<<"drone "<<drone_id<<" ..."<<"\n";
        }


        // Find heuristic (Eucledian distance)
        float find_h(const float x, const float y,const bool forward){
            if(forward){
                return sqrt(pow(goal_point.first - x, 2) + pow(goal_point.second - y, 2));
                }
            else{
                return sqrt(pow(start_point.first - x, 2) + pow(start_point.second - y, 2));
                }
        }


        std::mutex m;


        // Explores the current cell and adds it open_list if required
        void expand_cell(float x, float y, std::map<std::pair<float,float>,std::vector<float>>& all_points, gp::pq& open_points,
            std::set<std::pair<float,float>>& closed_points, std::pair<float,float>& parent, float distance, bool forward){
            if(is_valid(x,y) && is_not_obstacle(x, y, all_points) && !is_in_closed_set(x, y, closed_points)){
                float curr_h = find_h(x,y,forward);
                float curr_g = all_points[parent][4] + distance;
                //weighted a star part
                float curr_f = (curr_g <= curr_h)?curr_g+curr_h:curr_g + ((2*gp::w-1)*curr_h)/gp::w;
                //float curr_f = curr_g + curr_h;
                if(curr_f < all_points[{x,y}][2]){   
                    all_points[{x,y}] = {parent.first, parent.second, curr_f, curr_h, curr_g, 0};
                    open_points.push({curr_f, x, y});
                    //m.lock();
                    if(forward)
                        gp::visited_fwd.insert({x,y});
                    else
                        gp::visited_bwd.insert({x,y});
                    //m.unlock();
                }
            }
        }


        // Planning using A*
        void plan(std::map<std::pair<float,float>,std::vector<float>>& all_points, gp::pq& open_points, 
            std::set<std::pair<float,float>>& closed_points, bool forward){
            while(!open_points.empty()){
                std::vector<float> curr = open_points.top();
                open_points.pop();
                std::pair<float, float> curr_point = {curr[1],curr[2]};

                //m.lock();
                if(gp::found)
                    return;
                if((gp::visited_fwd.find(curr_point) != gp::visited_fwd.end() && 
                    gp::visited_bwd.find(curr_point) != gp::visited_bwd.end())){
                    std::cout<<"Path Found for "<<"drone "<<drone_id<<" ..."<<"\n"; // and explored: "<<closed_points.size()+open_points.size()<<" "<<forward<<std::endl;
                    //std::cout<<"Meet point "<<curr[1]<<" "<<curr[2]<<std::endl;
                    gp::meet_point = curr_point;
                    gp::found = true;
                    break;
                }
                if(is_in_closed_set(curr[1], curr[2], closed_points))
                    continue;
                closed_points.insert(curr_point);

                // explores all the 8 neighbours
                expand_cell(curr[1]+1, curr[2], all_points, open_points, closed_points, curr_point, 1., forward);
                expand_cell(curr[1]+1, curr[2]-1, all_points, open_points, closed_points, curr_point, 1.414f, forward);
                expand_cell(curr[1]+1, curr[2]+1, all_points, open_points, closed_points, curr_point, 1.414, forward);
                expand_cell(curr[1], curr[2]+1, all_points, open_points, closed_points, curr_point, 1., forward);
                expand_cell(curr[1], curr[2]-1, all_points, open_points, closed_points, curr_point, 1., forward);
                expand_cell(curr[1]-1, curr[2], all_points, open_points, closed_points, curr_point, 1., forward);
                expand_cell(curr[1]-1, curr[2]-1, all_points, open_points, closed_points, curr_point, 1.414f, forward);
                expand_cell(curr[1]-1, curr[2]+1, all_points, open_points, closed_points, curr_point, 1.414f, forward);
            }
        }



        // Trace node path from the intersection point of forward and backward searches
        void trace_path(std::map<std::pair<float,float>,std::vector<float>>& all_points_fwd, 
            std::map<std::pair<float,float>,std::vector<float>>& all_points_bwd){
                std::vector<std::pair<float,float>> path;
                float x = gp::meet_point.first;
                float y = gp::meet_point.second;
                path.push_back({x,y});

                // Store path length
                path_len = all_points_fwd[{x,y}][4]+all_points_bwd[{x,y}][4];

                // std::cout<<"Path length for "<<"drone "<<drone_id<<": "<<path_len<<std::endl;

                while(x != start_point.first || y != start_point.second){
                    float p1 = all_points_fwd[{x,y}][0];
                    float p2 = all_points_fwd[{x,y}][1];
                    x = p1;
                    y = p2;
                    path.push_back({x,y});
                }

                reverse(path.begin(),path.end());

                x = gp::meet_point.first;
                y = gp::meet_point.second;
                while(x != goal_point.first || y != goal_point.second){
                    float p1 = all_points_bwd[{x,y}][0];
                    float p2 = all_points_bwd[{x,y}][1];
                    x = p1;
                    y = p2;
                    path.push_back({x,y});
                }

                std::cout<<"Solution path for "<<"drone "<<drone_id<<" ..."<<"\n";

                // Print node path chosen by Bi-A*
                for(auto x: path)
                {
                    std::cout<<x.first<<" "<<x.second<<std::endl;

                    // Store nodes
                    nodes.push_back(x.first);
                    nodes.push_back(x.second);
                }

                // Store path length at the end
                nodes.push_back(path_len);

                // Store drone position at the end
                nodes.push_back(xy0.first);
                nodes.push_back(xy0.second);

            // Send plan to Survey
            send_to_survey();

            }


        void send_to_survey()
        {
            // Send node array to publisher
            auto msg = std_msgs::msg::Int32MultiArray();
            msg.data = nodes;

            publisher_->publish(msg);

            // std::cout<<"Sent"<<drone_id<<" position to survey..."<<"\n";
        }

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;


};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanning>());
    rclcpp::shutdown();
    return 0;
}