#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "robot_msgs/delivery.h"
#include "robot_msgs/ui_show.h"
#include "robot_msgs/ui_get.h"
#include "robot_msgs/pick.h"
#include "robot_msgs/Door_open.h"
#include "robot_msgs/deliveryAction.h" 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include "tracer_msgs/TracerStatus.h"
#include "tracer_msgs/TracerLightCmd.h"

#include <vector>
#include <string>
#include <sstream>
#include <random>

#include <thread>
#include <condition_variable>

#include <algorithm>

// 根据实际消息类型和服务类型包含头文件

class TaskManagerNode {

private:

    // 存储待配送目标的结构
    struct DeliveryTask {
        int bin_number;           // 箱号
        std::vector<int> address; // 地址数组 [building, unit, floor, room]
    };
    std::vector<DeliveryTask> task_list_;    // 待处理任务列表
    DeliveryTask current_task_;              // 当前任务
    std::vector<DeliveryTask> rest_task_;    // 剩余任务

    void assignTask(const std::vector<DeliveryTask>& tasks);
    void taskAssignLoop();
    void sortTaskList();
    std::mutex task_list_mutex_; 
    std::condition_variable task_cv_;

    // 验证码
    ros::ServiceClient pickup_client_; // 新增pickup服务客户端
    int pickup_code_ = 1234; // 假设取件码

    //机器人状态

    int battery_ = 0; // 电池电量
    float speed_ = 0; // 速度
    double odometry_ = 0.0; // 里程计
    double working_time_ = 0.0; // 工作时间
    std::string network_ = "Good"; // 网络状态
    int task_process_ = 0; // 任务进度（0-100）
    std::string task_status_ = "waitting..."; // 任务状态
    std::string current_task_show_ = ""; //
    std::vector<std::string> rest_task_show_; // 剩余任务（数组）

    void robot_status_update();

    // 功能函数
    void robot_voice(int num);
    void robot_calling(const std::string& msg);
    void pickup_code_generation();

    // 发布者
    ros::Publisher tracer_light_pub_;
    ros::Publisher speed_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher ui_show_pub_;
    ros::Publisher speach_client_;
    ros::Publisher calling_client_;

    void publishUiShowLoop();

    // 订阅者
    ros::Subscriber tracer_status_sub_;
    ros::Subscriber navigation_status_sub_;
    ros::Subscriber ui_door_open_sub_;

    // 客户端
    ros::ServiceClient delivery_cmd_client_;
    ros::ServiceClient delivery_door_open_client_;

    robot_msgs::delivery delivery_req;

    bool push_out(int num);
    bool door_open(int num);
    bool bigDoorOpen();
    bool door_ir_control(const std::string& status);
    bool Is_door_close = true;

    // 服务端
    ros::ServiceServer ui_get_server_;

    // 动作客户端
    std::shared_ptr<actionlib::SimpleActionClient<robot_msgs::deliveryAction>> delivery_ac_;

    // 服务通信客户端调用函数
    bool deliveryCmd(robot_msgs::delivery& req);

    // 动作客户端调用函数
    void sendDeliveryGoal(const DeliveryTask& task);
    void sendGoBackGoal(const std::vector<int>& task);

    // 回调函数声明
    void tracerStatusCallback(const tracer_msgs::TracerStatus::ConstPtr& msg);
    void navigationStatusCallback(const std_msgs::String::ConstPtr& msg);
    void doorOpenCallback(const robot_msgs::Door_open::ConstPtr& msg);

    bool uiGetCallback(robot_msgs::ui_get::Request& req, robot_msgs::ui_get::Response& res);


public:

TaskManagerNode(ros::NodeHandle& nh);

void pub_setup();
void client_setup();
void action_client_setup();
void taskAssign_setup();
    
};