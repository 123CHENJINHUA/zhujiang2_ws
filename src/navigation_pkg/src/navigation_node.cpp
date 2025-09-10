#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_msgs/deliveryAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <map>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <sstream>

class DeliveryActionServer
{
public:
    DeliveryActionServer(ros::NodeHandle& nh)
        : as_(nh, "delivery_action", boost::bind(&DeliveryActionServer::executeCB, this, _1), false),
          move_base_client_("move_base", true) // Initialize move_base action client
    {
        // Start the delivery action server
        as_.start();
        ROS_INFO("Delivery Action Server started.");

        // Set default initial position (fallback if not found in YAML)
        init_position_ = {{0.087, -1.585, 0.0}, {0.0, 0.0, 0.99, -0.0135}}; // Default position and orientation
        
        // Load addresses from YAML file (this may override init_position_ if 0,0,0,0 exists)
        loadAddressesFromYAML();
    }

    void loadAddressesFromYAML()
    {
        try {
            // Get the package path for unique_move_pkg
            std::string pkg_path = ros::package::getPath("navigation_pkg");
            std::string yaml_path = pkg_path + "/config/addresses.yaml";
            
            // Load the YAML file
            YAML::Node addresses = YAML::LoadFile(yaml_path);
            
            for(const auto& address : addresses) {
                std::string name = address.first.as<std::string>();
                YAML::Node pos = address.second;
                
                // Parse position and rotation from YAML
                std::vector<double> position = {
                    pos["x"].as<double>(),
                    pos["y"].as<double>(),
                    pos["z"].as<double>()
                };
                
                std::vector<double> rotation = {
                    pos["rotation"]["x"].as<double>(),
                    pos["rotation"]["y"].as<double>(),
                    pos["rotation"]["z"].as<double>(),
                    pos["rotation"]["w"].as<double>()
                };
                
                // Parse the address format (e.g., "1,1,1,3")
                std::vector<int> addr_components;
                std::stringstream ss(name);
                std::string component;
                while (std::getline(ss, component, ',')) {
                    addr_components.push_back(std::stoi(component));
                }
                
                if (addr_components.size() == 4) {
                    // Create a key using the address components
                    AddressKey key = {
                        addr_components[0], // building
                        addr_components[1], // unit
                        addr_components[2], // floor
                        addr_components[3]  // room
                    };
                    
                    // Store in the map
                    address_positions_[key] = std::make_pair(position, rotation);
                    
                    // Check if this is the initial position (0,0,0,0)
                    if (addr_components[0] == 0 && addr_components[1] == 0 && 
                        addr_components[2] == 0 && addr_components[3] == 0) {
                        init_position_ = std::make_pair(position, rotation);
                        // ROS_INFO_STREAM("Loaded initial position from address " << name << 
                        //     ": x=" << position[0] << 
                        //     ", y=" << position[1] << 
                        //     ", z=" << position[2] <<
                        //     ", rotation: [" << rotation[0] << ", " << 
                        //     rotation[1] << ", " << rotation[2] << ", " << 
                        //     rotation[3] << "]");
                    } 
                    // else 
                    // {
                    //     ROS_INFO_STREAM("Loaded position for address " << name << 
                    //         ": x=" << position[0] << 
                    //         ", y=" << position[1] << 
                    //         ", z=" << position[2] <<
                    //         ", rotation: [" << rotation[0] << ", " << 
                    //         rotation[1] << ", " << rotation[2] << ", " << 
                    //         rotation[3] << "]");
                    // }
                }
            }
            ROS_INFO("Successfully loaded addresses from YAML file");
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error loading addresses from YAML: " << e.what());
        }
    }

    void executeCB(const robot_msgs::deliveryGoalConstPtr& goal)
    {

        if (goal->building == 0 && goal->unit == 0 && goal->floor == 0 && goal->room == 0)
        {
            ROS_INFO("Back to init pose.");
            target_position = init_position_.first;
            target_orientation = init_position_.second;
        }

        else
        {
            ROS_INFO("Received delivery goal: %dB%dU%dF%dR", goal->building, goal->unit, goal->floor, goal->room);

            // Create address key from the goal
            AddressKey key = {
                goal->building,
                goal->unit,
                goal->floor,
                goal->room
            };

            // Find the target position for the address
            auto it = address_positions_.find(key);
            if (it == address_positions_.end())
            {
                ROS_ERROR("Address %d,%d,%d,%d not found in mapping!", 
                    goal->building, goal->unit, goal->floor, goal->room);
                robot_msgs::deliveryResult result;
                result.success = false;
                result.info = "Address not found";
                as_.setAborted(result);
                return;
            }
    
            target_position = it->second.first;
            target_orientation = it->second.second;
            
        }

        // Wait for move_base action server to be available
        if (!move_base_client_.waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("move_base action server not available!");
            robot_msgs::deliveryResult result;
            result.success = false;
            result.info = "Navigation server unavailable";
            as_.setAborted(result);
            return;
        }

        // Create and send navigation goal
        move_base_msgs::MoveBaseGoal nav_goal;
        nav_goal.target_pose.header.frame_id = "map_3d";
        nav_goal.target_pose.header.stamp = ros::Time::now();
        nav_goal.target_pose.pose.position.x = target_position[0];
        nav_goal.target_pose.pose.position.y = target_position[1];
        nav_goal.target_pose.pose.position.z = target_position[2];
        nav_goal.target_pose.pose.orientation.x = target_orientation[0];
        nav_goal.target_pose.pose.orientation.y = target_orientation[1];
        nav_goal.target_pose.pose.orientation.z = target_orientation[2];
        nav_goal.target_pose.pose.orientation.w = target_orientation[3];

        ros::Duration(5).sleep();
        move_base_client_.sendGoal(nav_goal);

        // Publish constant feedback
        robot_msgs::deliveryFeedback feedback;
        feedback.status = 100; // Constant feedback value
        as_.publishFeedback(feedback);

        // Monitor navigation progress with infinite waiting
        while (true)
        {
            actionlib::SimpleClientGoalState state = move_base_client_.getState();

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Navigation to room %d succeeded.", goal->room);

                // Set result
                robot_msgs::deliveryResult result;
                result.success = true;
                result.info = "Finish";
                as_.setSucceeded(result);
                ROS_INFO("Delivery succeeded.");
                return;
            }
            else if (state == actionlib::SimpleClientGoalState::ABORTED || state == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                ROS_ERROR("Navigation to room %d failed. State: %s", goal->room, state.toString().c_str());

                robot_msgs::deliveryResult result;
                result.success = false;
                result.info = "Navigation failed";
                as_.setAborted(result);
                return;
            }
            else if (state == actionlib::SimpleClientGoalState::ACTIVE)
            {
                ROS_INFO("Navigation to room %d is still active. Waiting for completion...", goal->room);
            }

            // Sleep for a short duration to avoid busy-waiting
            ros::Duration(1.0).sleep();
        }
    }

private:
    // Structure to hold address components
    struct AddressKey {
        int building;
        int unit;
        int floor;
        int room;

        bool operator<(const AddressKey& other) const {
            return std::tie(building, unit, floor, room) <
                   std::tie(other.building, other.unit, other.floor, other.room);
        }
    };

    actionlib::SimpleActionServer<robot_msgs::deliveryAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    std::map<AddressKey, std::pair<std::vector<double>, std::vector<double>>> address_positions_; // Address-to-position mapping
    std::pair<std::vector<double>, std::vector<double>> init_position_; // Initial position and orientation
    std::vector<double> target_position;
    std::vector<double> target_orientation;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delivery_as_node");
    ros::NodeHandle nh;
    DeliveryActionServer server(nh);
    ros::spin();
    return 0;
}