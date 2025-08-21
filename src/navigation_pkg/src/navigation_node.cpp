#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_msgs/deliveryAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <map>

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

        // Initialize room-to-position mapping
        room_positions_ = {
            {1, {{5.49, -1.1, 0.0}, {0.0, 0.0, -0.0098, 0.99}}}, // Position: x, y, z; Orientation: x, y, z, w
            {2, {{13.14, -0.52, 0.0}, {0.0, 0.0, -0.0027, 0.99}}},
            {3, {{21.60, 2.91, 0.0}, {0.0, 0.0, 0.69, 0.72}}},
            {4, {{6.91, 2.53, 0.0}, {0.0, 0.0, 0.71, 0.71}}}
            // Add more room mappings as needed
        };

        init_position_ = {{0.26, -0.89, 0.0}, {0.0, 0.0, 0.99, -0.015}}; // Default position and orientation
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

            // Find the target position for the room
            auto it = room_positions_.find(goal->room);
            if (it == room_positions_.end())
            {
                ROS_ERROR("Room %d not found in mapping!", goal->room);
                robot_msgs::deliveryResult result;
                result.success = false;
                result.info = "Room not found";
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
    actionlib::SimpleActionServer<robot_msgs::deliveryAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    std::map<int, std::pair<std::vector<double>, std::vector<double>>> room_positions_; // Room-to-position mapping
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