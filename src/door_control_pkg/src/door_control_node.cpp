#include <ros/ros.h>
#include <std_msgs/String.h>
#include "robot_msgs/delivery.h"
#include <thread>
#include <atomic>

class DoorControlNode
{
public:
    DoorControlNode(ros::NodeHandle& nh)
        : nh_(nh), is_sending_(false)
    {
        // 创建服务客户端
        delivery_cmd_client_ = nh_.serviceClient<robot_msgs::delivery>("/delivery_cmd");
        
        // 创建订阅者
        door_control_sub_ = nh_.subscribe("/door_control", 10, &DoorControlNode::doorControlCallback, this);
        
        ROS_INFO("DoorControlNode initialized.");
        ROS_INFO("Waiting for /delivery_cmd service...");
        
        // 等待服务可用
        if (delivery_cmd_client_.waitForExistence(ros::Duration(10.0))) {
            ROS_INFO("/delivery_cmd service is ready.");
        } else {
            ROS_WARN("/delivery_cmd service not available after 10 seconds.");
        }
    }

    ~DoorControlNode()
    {
        // 停止发送线程
        stopSending();
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient delivery_cmd_client_;
    ros::Subscriber door_control_sub_;
    
    std::atomic<bool> is_sending_;
    std::thread sending_thread_;

    void doorControlCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Received door control command: %s", msg->data.c_str());
        
        if (msg->data == "open")
        {
            startSending();
        }
        else if (msg->data == "stop")
        {
            stopSending();
        }
        else
        {
            ROS_WARN("Unknown door control command: %s", msg->data.c_str());
        }
    }

    void startSending()
    {
        if (is_sending_.load())
        {
            ROS_INFO("Already sending open requests.");
            return;
        }
        
        ROS_INFO("Starting to send open requests every 1 second...");
        is_sending_.store(true);
        
        // 如果之前的线程还在运行，先等待它结束
        if (sending_thread_.joinable())
        {
            sending_thread_.join();
        }
        
        // 启动新的发送线程
        sending_thread_ = std::thread(&DoorControlNode::sendingLoop, this);
    }

    void stopSending()
    {
        if (!is_sending_.load())
        {
            return;
        }
        
        ROS_INFO("Stopping open requests...");
        is_sending_.store(false);
        
        // 等待发送线程结束
        if (sending_thread_.joinable())
        {
            sending_thread_.join();
        }
        
        ROS_INFO("Open requests stopped.");
    }

    void sendingLoop()
    {
        ros::Rate rate(0.2); // 1Hz - 每秒发送一次
        
        while (is_sending_.load() && ros::ok())
        {
            // 创建服务请求
            robot_msgs::delivery srv;
            srv.request.delivery_msgs = "remoteControl";
            
            // 调用服务
            if (delivery_cmd_client_.call(srv))
            {
                if (srv.response.status_msgs)
                {
                    ROS_INFO("Door open request sent successfully.");
                }
                else
                {
                    ROS_WARN("Door open request failed on server side.");
                }
            }
            else
            {
                ROS_ERROR("Failed to call /delivery_cmd service.");
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_control_node");
    ros::NodeHandle nh;
    
    DoorControlNode door_control_node(nh);
    
    ROS_INFO("Door control node is running...");
    
    ros::spin();
    
    return 0;
}
