#ifndef DOOR_CONTROL_NODE_H
#define DOOR_CONTROL_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "robot_msgs/delivery.h"
#include <thread>
#include <atomic>

class DoorControlNode
{
public:
    DoorControlNode(ros::NodeHandle& nh);
    ~DoorControlNode();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient delivery_cmd_client_;
    ros::Subscriber door_control_sub_;
    
    std::atomic<bool> is_sending_;
    std::thread sending_thread_;

    void doorControlCallback(const std_msgs::String::ConstPtr& msg);
    void startSending();
    void stopSending();
    void sendingLoop();
};

#endif // DOOR_CONTROL_NODE_H
