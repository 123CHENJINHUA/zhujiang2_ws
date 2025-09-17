#include <ros/ros.h>
#include "robot_msgs/delivery.h"
#include "master_pkg/tcp_server.h"

TcpServer* g_tcp_server = nullptr;

bool deliveryCmdCallback(robot_msgs::delivery::Request& req, robot_msgs::delivery::Response& res) {
    if (g_tcp_server) {
        // ROS_INFO("Sending command to MCU: %s", req.delivery_msgs.c_str());
        // 发送消息前先清空 socket 缓冲区
        g_tcp_server->clearSocketBuffer();
        // 发送消息前检查连接状态
        if (!g_tcp_server->sendMessage(req.delivery_msgs)) {
            ROS_ERROR("Failed to send message to MCU: %s", req.delivery_msgs.c_str());
            res.status_msgs = false;
            return true;
        }

        // 根据命令内容选择不同的超时
        int timeout = g_tcp_server->timeout_delivery_cmd;
        if (req.delivery_msgs.find("door") != std::string::npos || 
            req.delivery_msgs.find("bigDoorOpen") != std::string::npos) {
            timeout = g_tcp_server->timeout_door_open;
        }

        std::string reply = g_tcp_server->receiveMessage(timeout);
        if (reply.empty()) {
            ROS_ERROR("Timeout or no response for: %s", req.delivery_msgs.c_str());
            res.status_msgs = false;
            return true;
        }
        ROS_INFO("MCU reply: %s", reply.c_str());
        res.status_msgs = true;
        return true;
    }
    ROS_ERROR("tcp server not available");
    res.status_msgs = false;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "delivery_node");
    ros::NodeHandle nh;

    int server_port = 5001;
    std::string allowed_ip = "192.168.0.111";

    TcpServer tcp_server(server_port,allowed_ip);
    if (!tcp_server.start()) {
        ROS_ERROR("Failed to start TCP server.");
        return -1;
    }
    ROS_INFO("Waiting for MCU(client) to connect...");
    if (!tcp_server.acceptClient()) {
        ROS_ERROR("Failed to accept client.");
        return -1;
    }
    ROS_INFO("MCU connected.");
    g_tcp_server = &tcp_server;

    ros::ServiceServer delivery_cmd_server = nh.advertiseService("/delivery_cmd", deliveryCmdCallback);
    ROS_INFO("Service /delivery_cmd is ready.");
    ros::spin();
    tcp_server.closeSocket();
    return 0;
}