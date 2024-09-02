// This node first creates a randomly weighted 2D graph for use in motion planning for a drone.

// Then, this node applies Bidirectional A* to find a time optimal path from a source to a destination.

// The code for Bi-A* is adapted from https://github.com/akshay-antony/BiDirectionalWeightedAstar/blob/main/src/main.cpp

#include "uav.h"

#include <memory>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

// Node class
class MissionPlanning : public rclcpp::Node
{

    public:

        // Declare start and goal points, and ID, for the drone
        std::pair<float,float> start_point;
        std::pair<float,float> goal_point;
        int drone_id;

        std::vector<int> nodes;
        u_int32_t path_len;

        // Constructor
        MissionPlanning()
        : Node("mission_plan")
        {

            // Declare parameters with defaults
            this->declare_parameter("start_x",1);
            this->declare_parameter("start_y",1);
            this->declare_parameter("goal_x",30);
            this->declare_parameter("goal_y",30);
            this->declare_parameter("drone_id",0);

            // Retrive actual parameters
            start_point.first = (this->get_parameter("start_x")).as_int();
            start_point.second = (this->get_parameter("start_y")).as_int();
            goal_point.first = (this->get_parameter("goal_x")).as_int();
            goal_point.second = (this->get_parameter("goal_x")).as_int();
            drone_id = (this->get_parameter("drone_id")).as_int();

            // Declare publisher for node path
            publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("node_path",10);

            path_len = 0;

            // Plan mission path
            plan_mission();

        }


    private:

        int plan_mission()
        {
            // Start timing solve duration
            auto start_time = std::chrono::high_resolution_clock::now();

            // Add occupied cells.
            // Real-time positions of other drones can be treated as occupied cells
            std::vector<std::pair<int,int>> input_obstacles;
            input_obstacles.push_back({2, 2});
            input_obstacles.push_back({2, 3});

            // Each cell of the map contains {x_cell, y_cell, f, h, g, is_occ}
            std::map<std::pair<float,float>,std::vector<float>> all_points_fwd, all_points_bwd;

            gp::pq open_points_fwd, open_points_bwd;

            std::set<std::pair<float,float>> closed_points_fwd, closed_points_bwd;

            // bool found = false;

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

            if(gp::found)
                trace_path(all_points_fwd, all_points_bwd);
            else
                std::cout<<"No path Found"<<std::endl;

            // Stop clock for solve-time
            auto stop_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time-start_time);
            
            std::cout<<"Path planning time for drone "<<drone_id<<" (in milliseconds): "<<duration.count()<<std::endl;

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

                // std::cout<<"Number of nodes for "<<"drone "<<path.size()<<" ..."<<"\n";

                // Print node path chosen by Bi-A*
                for(auto x: path)
                {
                    std::cout<<x.first<<" "<<x.second<<std::endl;

                    // Store nodes
                    nodes.push_back(x.first);
                    nodes.push_back(x.second);
                }

                // Store path length  at the end
                nodes.push_back(path_len);

            // Send plan to Survey
            send_to_survey();

            }


        void send_to_survey()
        {
            // Send node array to publisher
            auto msg = std_msgs::msg::Int32MultiArray();
            msg.data = nodes;

            // msg.layout.dim.size = path_len;
            while(true)
            {
                publisher_->publish(msg);
            }
        }

        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;


};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionPlanning>());
    rclcpp::shutdown();
    return 0;
}