#!/home/cjh/miniconda3/envs/zhujiang2/bin/python

import sys
sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang2/lib/python3.10/site-packages")
sys.path.insert(0, "/home/cjh/zhujiang2_ws/src")

import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from robot_msgs.srv import unique_move, unique_moveResponse
from unique_move_pkg.scripts.trajectory import CarTrajectory
import numpy as np
import cv2
import threading
import math
from scipy.stats import mode
from robot_msgs.msg import Marker2dPose
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, actual):
        error = target - actual
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class UniqueMoveNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('unique_move_node')
        
        # 创建发布者，发布到/cmd_vel话题，消息类型为Twist
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 创建新的发布者，发布到/aruco_marker_info话题，消息类型为String
        self.marker_info_pub = rospy.Publisher('/aruco_marker_info', String, queue_size=10)
        
        # 创建服务，服务类型为MoveRobot，回调函数为handle_move_request
        self.service = rospy.Service('/unique_move_service', unique_move, self.handle_move_request)

        self.car = CarTrajectory(x=0,y=-0.71, theta=0)

        self.IsLocated = False  # 是否定位成功
        self.marker_pose = None
        self.body_foot_pose = None

        self.linear_pid = PIDController(kp=0.5, ki=0.00, kd=0.0)  # PID for linear velocity
        self.angular_pid = PIDController(kp=0.05, ki=0.00, kd=0.0)  # PID for angular velocity
        self.initial_pose = None  # Store the initial pose
        self.body_foot_pose = None  # Store the latest /body_foot_2d_pose data
        rospy.Subscriber('/body_foot_2d_pose', Marker2dPose, self.update_body_foot_pose)
        

        # 参数设置
        self.linear_x = 0.1  # 线速度(m/s)
        self.angular_z = 0.2  # 角速度(rad/s)
        self.distance2wall = None
        self.angle2wall = None
        self.center2edge_offset = self.car.W / 2.0  # 小车中心到边缘的偏移量
        self.camera_angle_offset = 0
        self.wall2edge_offset = 0.0

        rospy.loginfo("Unique Move Service is ready...")

    def detect_to_wall_distance(self):
        """
        通过订阅 /marker_2d_pose 和 /body_foot_2d_pose 计算距离和角度
        """
        try:

            if not self.IsLocated:
                # 收集 20 次 /marker_2d_pose 数据
                marker_poses = []
                for _ in range(20):
                    try:
                        marker_pose = rospy.wait_for_message('/marker_2d_pose', Marker2dPose, timeout=1.0)
                        marker_poses.append(marker_pose)
                    except rospy.ROSException as e:
                        rospy.logwarn(f"Failed to get marker pose: {str(e)}")
                        continue
                    rospy.sleep(0.05)  # 小延迟

                if not marker_poses:
                    rospy.logwarn("Failed to collect sufficient marker pose data.")
                    return None, None

                # 计算 marker_pose 的平均值
                avg_x = sum(pose.x for pose in marker_poses) / len(marker_poses)
                avg_y = sum(pose.y for pose in marker_poses) / len(marker_poses)
                avg_theta = sum(pose.theta for pose in marker_poses) / len(marker_poses)

                self.marker_pose = Marker2dPose(x=avg_x, y=avg_y, theta=avg_theta)
                self.IsLocated = True  # 定位成功

            # 获取 /body_foot_2d_pose 的数据
            self.body_foot_pose = rospy.wait_for_message('/body_foot_2d_pose', Marker2dPose, timeout=1.0)

            if self.marker_pose is None or self.body_foot_pose is None:
                rospy.logwarn("Marker or body foot pose not received yet.")
                return None, None

            # 计算 marker_pose 的直线方程 y = kx + b
            marker_theta_rad = math.radians(self.marker_pose.theta)  # 将角度转换为弧度
            k = math.tan(marker_theta_rad)  # 斜率
            b = self.marker_pose.y - k * self.marker_pose.x  # 截距

            # 计算 body_foot_pose 点到直线的垂直距离
            x0, y0 = self.body_foot_pose.x, self.body_foot_pose.y
            detect_distance = abs(k * x0 - y0 + b) / math.sqrt(k**2 + 1)

            # 计算夹角
            marker_theta = math.radians(self.marker_pose.theta)
            body_foot_theta = math.radians(self.body_foot_pose.theta)
            detect_angle = math.degrees(marker_theta - body_foot_theta)

            # rospy.loginfo(f"Detected distance: {detect_distance:.2f} m, Detected angle: {detect_angle:.2f}°")
            return detect_distance, detect_angle

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to get pose data: {str(e)}")
            return None, None


    def update_body_foot_pose(self, msg):
        """
        Callback to update the latest /body_foot_2d_pose data.
        """
        self.body_foot_pose = msg

    def set_initial_pose(self):
        """
        Record the initial pose from /body_foot_2d_pose.
        """
        if self.body_foot_pose is None:
            rospy.logwarn("Body foot pose not received yet.")
            return False
        self.initial_pose = self.body_foot_pose
        return True

    def calculate_actual_distance_and_angle(self):
        """
        Calculate the actual distance and angle based on the initial and current pose.
        """
        if self.initial_pose is None or self.body_foot_pose is None:
            rospy.logwarn("Initial or current pose not available.")
            return None, None

        # Calculate actual distance
        dx = self.body_foot_pose.x - self.initial_pose.x
        dy = self.body_foot_pose.y - self.initial_pose.y
        actual_distance = math.sqrt(dx**2 + dy**2)

        # Calculate actual angle
        actual_angle = self.body_foot_pose.theta - self.initial_pose.theta

        return actual_distance, actual_angle

    def publish_linear_velocity(self, target_distance):
        """
        发布线速度指令，使用PID控制
        :param target_distance: 目标前进距离
        """
        if not self.set_initial_pose():
            rospy.logwarn("Failed to set initial pose.")
            return

        vel_msg = Twist()
        rate = rospy.Rate(20)  # 设置发布频率为20Hz

        while True:
            # Calculate actual distance
            actual_distance, _ = self.calculate_actual_distance_and_angle()
            actual_distance = actual_distance * target_distance / abs(target_distance) if target_distance != 0 else 0
            if actual_distance is None:
                continue

            # Compute PID output
            pid_output = self.linear_pid.compute(target_distance, actual_distance)
            vel_msg.linear.x = max(min(pid_output, self.linear_x), -self.linear_x)  # 限制速度范围

            self.vel_pub.publish(vel_msg)
            rate.sleep()

            actual_distance, _ = self.calculate_actual_distance_and_angle()
            actual_distance = actual_distance * target_distance / abs(target_distance) if target_distance != 0 else 0
            if actual_distance is None:
                continue

            # Stop if the error is small enough
            if abs(target_distance - actual_distance) < 0.01:
                break

        # 停止机器人
        vel_msg.linear.x = 0.0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)

    def publish_angular_velocity(self, target_angle):
        """
        发布角速度指令，使用PID控制
        :param target_angle: 目标旋转角度
        """
        if not self.set_initial_pose():
            rospy.logwarn("Failed to set initial pose.")
            return

        vel_msg = Twist()
        rate = rospy.Rate(20)  # 设置发布频率为20Hz

        while True:
            # Calculate actual angle
            _, actual_angle = self.calculate_actual_distance_and_angle()
            if actual_angle is None:
                continue

            # Compute PID output
            pid_output = self.angular_pid.compute(target_angle, actual_angle)
            vel_msg.angular.z = max(min(pid_output, self.angular_z), -self.angular_z)  # 限制角速度范围

            self.vel_pub.publish(vel_msg)
            rate.sleep()

            _, actual_angle = self.calculate_actual_distance_and_angle()
            if actual_angle is None:
                continue

            # Stop if the error is small enough
            if abs(target_angle - actual_angle) < 0.5:
                break

        # 停止机器人
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)

    def cal_average_distance_and_angle(self):
        """
        计算距离和角度的平均值
        """
        distances = []
        angles = []

        # Collect 20 samples of distance and angle
        for _ in range(10):
            distance2wall, angle2wall = self.detect_to_wall_distance()
            if distance2wall is not None and angle2wall is not None:
                distances.append(distance2wall)
                angles.append(angle2wall)
            rospy.sleep(0.05)
        if not distances or not angles:
            rospy.logwarn("Failed to collect sufficient data for averaging.")
            return None, None
        # Calculate averages
        avg_distance2wall = sum(distances) / len(distances)
        avg_angle2wall = sum(angles) / len(angles)
        rospy.loginfo(f"Average Distance to wall: {avg_distance2wall:.2f} m, Average Angle to wall: {avg_angle2wall:.2f}°")
        return avg_distance2wall, avg_angle2wall
        
    
    def handle_move_request(self, req):
        """
        服务回调函数，处理移动请求
        """
        response = unique_moveResponse()
        if req.order == "move":
            try:
                last_angle = 0  # 基础旋转角度(度)
                new_angle = 0    # 每次旋转角度增量(度)
                move_distance = self.car.D  # 每次前进距离
                step = 0
                self.IsLocated = False  # 重置定位状态

                while True:
                    
                    avg_distance2wall, avg_angle2wall = self.cal_average_distance_and_angle()
                    if avg_distance2wall is None or avg_angle2wall is None:
                        continue  # 如果未检测到有效的距离或角度，跳过当前循环

                    # Update car's distance and angle
                    self.car.d = avg_distance2wall - self.center2edge_offset - self.wall2edge_offset
                    self.car.angle2wall = avg_angle2wall - self.camera_angle_offset

                    if self.car.d <= self.car.minimum_distance:
                        # 回正 
                        current_angle = self.car.angle2wall    
                        self.car.rotate(current_angle)
                        self.publish_angular_velocity(current_angle)
                        rospy.loginfo(f"Step {step+1}: Rotated {current_angle:.1f}°")
                        
                        avg_distance2wall, avg_angle2wall = self.cal_average_distance_and_angle()
                        if avg_distance2wall is None or avg_angle2wall is None:
                            continue  # 如果未检测到有效的距离或角度，跳过当前循环

                        # Update car's distance and angle
                        self.car.d = avg_distance2wall - self.center2edge_offset - self.wall2edge_offset
                        self.car.angle2wall = avg_angle2wall - self.camera_angle_offset

                        rospy.loginfo(f"Final Distance to wall: {self.car.d:.2f} m, Angle to wall: {self.car.angle2wall:.2f}°")
                        break

                    rospy.loginfo(f"Step {step+1}: Distance to wall: {self.car.d:.2f} m, Angle to wall: {self.car.angle2wall:.2f}°")

                    if self.car.d > self.car.maximum_distance:
                        continue

                    # 调用 solve_theta 方法并获取返回值
                    valid_solutions = self.car.solve_theta()
                    if not valid_solutions:
                        rospy.loginfo("No valid solutions, using default angle of 45 degrees")
                        new_angle = 45
                    else:
                        # 使用第一个有效解作为旋转角度
                        new_angle = valid_solutions[0] * self.car.k * 180 / np.pi
                        rospy.loginfo(f"Step {step+1}: New angle calculated: {new_angle:.2f}°")
                        if new_angle > 45:
                            new_angle = 45

                    # 扭正角度
                    correct_angle = self.car.angle2wall
                    
                    #需要旋转的角度
                    current_angle = new_angle + correct_angle

                    # 旋转
                    self.car.rotate(current_angle)
                    self.publish_angular_velocity(current_angle)
                    rospy.loginfo(f"Step {step+1}: Rotated {current_angle:.1f}°")
                    
                    # 前进
                    self.car.move_forward(move_distance)
                    self.publish_linear_velocity(move_distance)
                    rospy.loginfo(f"Step {step+1}: Moved {move_distance} m")

                    current_angle = new_angle

                    # 旋转
                    self.car.rotate(-2 * current_angle)
                    self.publish_angular_velocity(-2 * current_angle)
                    rospy.loginfo(f"Step {step+1}: Rotated {-2 * current_angle:.1f}°")
                    
                    # 前进
                    self.car.move_forward(-move_distance)
                    self.publish_linear_velocity(-move_distance)
                    rospy.loginfo(f"Step {step+1}: Moved {move_distance} m)")

                    step += 1

                response.success = True
            except Exception as e:
                rospy.logerr(f"Failed to publish velocity command: {str(e)}")
                response.success = False
                response.message = str(e)
            
            return response

if __name__ == "__main__":
    try:
        node = UniqueMoveNode()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")