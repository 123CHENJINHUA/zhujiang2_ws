#include "master_pkg/task_manager_node.h"

TaskManagerNode::TaskManagerNode(ros::NodeHandle& nh)
    // : navigation_to_pose_ac_("/navigation_to_pose", true) // 动作客户端初始化
{
    // 发布者
    tracer_light_pub_ = nh.advertise<tracer_msgs::TracerLightCmd>("/tracer_light_control", 10);
    speed_pub_ = nh.advertise<geometry_msgs::Twist>("/speed", 10);
    emergency_stop_pub_ = nh.advertise<std_msgs::Bool>("/emergency_stop", 10);
    ui_show_pub_ = nh.advertise<robot_msgs::ui_show>("/UI_show", 10);
    speach_client_ = nh.advertise<std_msgs::String>("/speach", 10);
    calling_client_ = nh.advertise<std_msgs::String>("/calling", 10);

    // 订阅者
    tracer_status_sub_ = nh.subscribe("/tracer_status", 10, &TaskManagerNode::tracerStatusCallback, this);
    navigation_status_sub_ = nh.subscribe("/navigation_status", 10, &TaskManagerNode::navigationStatusCallback, this);
    ui_door_open_sub_ = nh.subscribe("/ui_door_open", 10, &TaskManagerNode::doorOpenCallback, this);

    // 服务客户端
    delivery_cmd_client_ = nh.serviceClient<robot_msgs::delivery>("/delivery_cmd");
    pickup_client_ = nh.serviceClient<robot_msgs::pick>("/pickup");


    // 服务端
    ui_get_server_ = nh.advertiseService("/UI_get", &TaskManagerNode::uiGetCallback, this);

    // 动作客户端
    delivery_ac_.reset(new actionlib::SimpleActionClient<robot_msgs::deliveryAction>("delivery_action", true));

    ROS_INFO("TaskManagerNode initialized.");
}



// 机器人状态更新函数
void TaskManagerNode::robot_status_update() {

    working_time_ = 3600.0; // 假设工作时间为3600秒
    network_ = "Good"; // 假设网络状态良好

    // 用 current_task_ 和 rest_task_ 更新显示内容
    if (!current_task_.empty()) {
        current_task_show_ = std::to_string(current_task_[0]) + "栋" +
                             std::to_string(current_task_[1]) + "单元" +
                             std::to_string(current_task_[2]) + "层" +
                             std::to_string(current_task_[3]) + "号";
    } else {
        current_task_show_.clear();
    }

    rest_task_show_.clear();
    for (const auto& t : rest_task_) {
        std::string task_str = std::to_string(t[0]) + "栋" +
                               std::to_string(t[1]) + "单元" +
                               std::to_string(t[2]) + "层" +
                               std::to_string(t[3]) + "号";
        rest_task_show_.push_back(task_str);
    }
}

// 话题订阅实现
void TaskManagerNode::tracerStatusCallback(const tracer_msgs::TracerStatus::ConstPtr& msg) {
    battery_ = (msg->battery_voltage-22.5)/(26.5-22.5) * 100; // 电池电压范围为22.5V到26.5V
    speed_ = msg->linear_velocity*3.6;
    odometry_ = (msg->left_odomter + msg->right_odomter) / 2.0; // 里程计取平均值
}

void TaskManagerNode::navigationStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received navigation status: %s", msg->data.c_str());
}

void TaskManagerNode::doorOpenCallback(const robot_msgs::Door_open::ConstPtr& msg) {
    ROS_INFO("Received door open request for door %d", msg->door_num);
    Is_door_close = false;
    robot_voice(20);
    robot_voice(19);
    // 创建线程执行门控制操作
    std::thread door_control_thread([this]() {
        bigDoorOpen();
        Is_door_close = true;
    });
    door_control_thread.detach(); // 分离线程，让它独立运行
}

// 发布者设置
void TaskManagerNode::pub_setup() {
    // 等待订阅者
    ROS_INFO("Waiting for UI_show, speach, and calling subscribers...");
    while (ui_show_pub_.getNumSubscribers() == 0 ||
           speach_client_.getNumSubscribers() == 0 ||
           calling_client_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("UI_show, speach, and calling subscribers connected!");

    // 启动独立线程循环发布机器人状态
    std::thread pub_thread(&TaskManagerNode::publishUiShowLoop, this);
    pub_thread.detach();
}

//机器人状态发布实现
void TaskManagerNode::publishUiShowLoop() {
    ros::Rate rate(5); // 降低到5Hz，减少对系统资源的争夺
    while (ros::ok()) {

        // 更新机器人状态
        robot_status_update();
        // 发布机器人状态到UI
        robot_msgs::ui_show ui_msg;
        ui_msg.battery =  battery_;
        ui_msg.odometry = odometry_;
        ui_msg.speed = speed_;
        ui_msg.working_time = working_time_;
        ui_msg.network = network_;
        ui_msg.task_status = task_status_;
        ui_msg.task_process = task_process_; // 任务进度
        ui_msg.current_task = current_task_show_;
        ui_msg.rest_task = rest_task_show_;
        ui_show_pub_.publish(ui_msg);
        rate.sleep();
    }
}

// 客户端设置
void TaskManagerNode::client_setup() {
    // 等待所有服务准备就绪
    ros::service::waitForService("/delivery_cmd");
    ros::service::waitForService("/pickup");
    ROS_INFO("All services are ready and action client is connected.");
}

// 客户端调用实现
bool TaskManagerNode::deliveryCmd(robot_msgs::delivery& req) {
    const int max_retries = 3;
    const ros::Duration retry_delay(0.5); // 500ms
    
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        if (delivery_cmd_client_.call(req)) {
            if (req.response.status_msgs == true) {
                ROS_INFO("Delivery cmd: %s : success (attempt %d)", req.request.delivery_msgs.c_str(), attempt);
                return true;
            }
            ROS_ERROR("Delivery cmd: %s : failed (attempt %d)", req.request.delivery_msgs.c_str(), attempt);
        } else {
            ROS_ERROR("Failed to execute delivery command: %s (attempt %d)", req.request.delivery_msgs.c_str(), attempt);
        }
        
        // 如果不是最后一次尝试，等待后重试
        if (attempt < max_retries) {
            ROS_WARN("Retrying delivery command in %f seconds...", retry_delay.toSec());
            retry_delay.sleep();
        }
    }
    
    ROS_ERROR("All attempts failed for command: %s", req.request.delivery_msgs.c_str());
    return false;
}

bool TaskManagerNode::push_out(int num) {
    delivery_req.request.delivery_msgs = "push " + std::to_string(num);
    return deliveryCmd(delivery_req);
}

bool TaskManagerNode::door_open(int num) {
    delivery_req.request.delivery_msgs = "door " + std::to_string(num);
    return deliveryCmd(delivery_req); 
}

bool TaskManagerNode::bigDoorOpen() {
    delivery_req.request.delivery_msgs = "bigDoorOpen";
    return deliveryCmd(delivery_req);
}

bool TaskManagerNode::door_ir_control(const std::string& status) {
    delivery_req.request.delivery_msgs = "ir " + status;
    return deliveryCmd(delivery_req);
}

// ui指令服务端回调实现
bool TaskManagerNode::uiGetCallback(robot_msgs::ui_get::Request& req, robot_msgs::ui_get::Response& res) {
    // 清空原有任务列表
    std::lock_guard<std::mutex> lock(task_list_mutex_); // 加锁
    task_list_.clear();

    // 解析请求字符串
    std::stringstream ss_groups(req.delivery_list.c_str());
    std::string group;
    while (std::getline(ss_groups, group, ';')) {
        std::vector<int> task;
        std::stringstream ss_values(group);
        std::string value;
        while (std::getline(ss_values, value, ',')) {
            try {
                task.push_back(std::stoi(value));
            } catch (...) {
                // 解析失败，返回错误
                res.received = false;
                return true;
            }
        }
        if (task.size() == 4) {
            task_list_.push_back(task);
        } else {
            res.received = false;
            return true;
        }
    }

    sortTaskList(); // 对任务列表进行排序

    task_cv_.notify_one(); // 唤醒分配线程
    res.received = true;
    ROS_INFO("Task list updated, size: %lu", task_list_.size());
    return true;
}

// 动作客户端调用实现

void TaskManagerNode::action_client_setup() {
    // 等待动作服务器连接
    if (!delivery_ac_->waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Failed to connect to navigation action server.");
        return;
    }
    ROS_INFO("Connected to navigation action server.");
}

void TaskManagerNode::sendDeliveryGoal(const std::vector<int>& task) {
    robot_msgs::deliveryGoal goal;
    goal.building = task[0];
    goal.unit = task[1];
    goal.floor = task[2];
    goal.room = task[3];

    auto feedback_cb = [this](const robot_msgs::deliveryFeedbackConstPtr& feedback) {
        task_process_ = feedback->status; // 更新任务状态
    };

    delivery_ac_->sendGoal(goal, 
        actionlib::SimpleActionClient<robot_msgs::deliveryAction>::SimpleDoneCallback(),
        actionlib::SimpleActionClient<robot_msgs::deliveryAction>::SimpleActiveCallback(),
        feedback_cb);

    robot_voice(100); // 100对应播放歌曲的指令，行驶过程中播放歌曲

    delivery_ac_->waitForResult();
    if (delivery_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        const robot_msgs::deliveryResultConstPtr& result = delivery_ac_->getResult();
        if (result) {
            task_status_ = result->info; // 将info赋值给current_task_show_
            ROS_INFO("Delivery succeeded! info: %s", result->info.c_str());
            // 新增：如果完成，调用pickup服务
            if (result->info == "Finish") {
                robot_msgs::pick srv;
                srv.request.pickup_code = pickup_code_;
                //语音提示
                robot_voice(6); // 7是语音提示取件码
                ROS_INFO("Waiting for pickup code input from UI...");
                if (pickup_client_.call(srv)) {
                    if (srv.response.success) {
                        while(!Is_door_close)
                        {
                            robot_voice(19);
                            ros::Duration(5.0).sleep();
                        }
                        robot_voice(18); // 18是语音提示取件成功
                        ROS_INFO("Pickup code correct, pickup success!");
                    } else {
                        robot_voice(15); // 13是语音提示取件失败
                        ROS_WARN("Pickup code failed 5 times or incorrect.");
                    }
                } else {
                    ROS_ERROR("Failed to call pickup service!");
                }
            }
        } else {
            ROS_INFO("Delivery succeeded! But result is null.");
        }
    } else {
        ROS_WARN("Delivery failed!");
    }
}


void TaskManagerNode::sendGoBackGoal(const std::vector<int>& task){
    robot_msgs::deliveryGoal goal;
    goal.building = task[0];
    goal.unit = task[1];
    goal.floor = task[2];
    goal.room = task[3];

    auto feedback_cb = [this](const robot_msgs::deliveryFeedbackConstPtr& feedback) {
        task_process_ = feedback->status; // 更新任务状态
    };

    delivery_ac_->sendGoal(goal, 
        actionlib::SimpleActionClient<robot_msgs::deliveryAction>::SimpleDoneCallback(),
        actionlib::SimpleActionClient<robot_msgs::deliveryAction>::SimpleActiveCallback(),
        feedback_cb);

    robot_voice(100); // 100对应播放歌曲的指令，行驶过程中播放歌曲

    delivery_ac_->waitForResult();
    if (delivery_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        const robot_msgs::deliveryResultConstPtr& result = delivery_ac_->getResult();
        if (result) {
            task_status_ = result->info; // 将info赋值给current_task_show_
            ROS_INFO("Delivery succeeded! info: %s", result->info.c_str());
            // 新增：如果完成，调用pickup服务
            if (result->info == "Finish") {
                robot_voice(17); // 17是我回来啦
            }
        } else {
            ROS_INFO("Delivery succeeded! But result is null.");
        }
    } else {
        ROS_WARN("Delivery failed!");
    }
}

// 任务分配设置
void TaskManagerNode::taskAssign_setup() {
    // 启动任务分配线程
    std::thread task_thread(&TaskManagerNode::taskAssignLoop, this);
    task_thread.detach();
}

// 任务分配函数
void TaskManagerNode::assignTask(const std::vector<std::vector<int>>& tasks) {
    if (!tasks.empty()) {
        current_task_ = tasks.front();
        rest_task_.clear();
        for (size_t i = 1; i < tasks.size(); ++i) {
            rest_task_.push_back(tasks[i]);
        }
    } else {
        current_task_.clear();
        rest_task_.clear();
    }
}

void TaskManagerNode::sortTaskList() {
    std::sort(task_list_.begin(), task_list_.end(),
        [](const std::vector<int>& a, const std::vector<int>& b) {
            return a < b; // 按字典序升序
        });
}

void TaskManagerNode::taskAssignLoop() {
    std::unique_lock<std::mutex> lock(task_list_mutex_);
    ROS_INFO("start delivery---.");
    while (ros::ok()) { 
        task_cv_.wait(lock, [this]{ return !task_list_.empty() || !ros::ok(); });
        robot_voice(14); // 14是语音提示，配送开始

        while (!task_list_.empty()) {

            // ros::Duration(1.0).sleep(); // 处理间隔
            // door_ir_control("on");
            // 生成新的取件码
            pickup_code_generation();
            //语音提示
            robot_voice(3); // 3 机器人正在通过
            //打电话
            robot_calling(std::to_string(pickup_code_));

            // door_open(4);

            // 发布任务分配信息
            task_status_ = "working";
            assignTask(task_list_);
            task_list_.erase(task_list_.begin());
            sendDeliveryGoal(current_task_);

            // door_ir_control("off");
        }

         // All tasks completed, return to original point
         ROS_INFO("All tasks completed. Returning to original point...");
         robot_voice(16); // 16是语音提示，返回原点
         std::vector<int> original_point = {0, 0, 0, 0}; // Define the original point
         sendGoBackGoal(original_point);
         task_status_ = "idle"; // Update status to idle
    }
}

// 功能模块
void TaskManagerNode::robot_voice(int num) {
    std_msgs::String msg;
    msg.data = std::to_string(num);
    speach_client_.publish(msg);
    ROS_INFO("Robot voice command sent: %s", msg.data.c_str());
}

void TaskManagerNode::robot_calling(const std::string& msg) {
    std_msgs::String calling_msg;
    calling_msg.data = msg;
    calling_client_.publish(calling_msg);
    ROS_INFO("Robot calling command sent: %s", msg.c_str());
}

void TaskManagerNode::pickup_code_generation() {
    // 每次分配任务前生成新的随机取件码（如4位数字）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1000, 9999);
    pickup_code_ = -1;
    // pickup_code_ = dis(gen);
}


// main函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "task_manager_node");
    ros::NodeHandle nh;
    TaskManagerNode node(nh);

    // 发布者设置
    node.pub_setup();
    // 客户端设置（单片机）
    node.client_setup();
    // 动作客户端设置
    node.action_client_setup();
    // 启动工作流
    node.taskAssign_setup();
    // 启动ROS事件循环
    ros::spin();
    return 0;
}