#!/home/cjh/miniconda3/envs/zhujiang2/bin/python
import numpy as np
from math import cos, sin

class CarTrajectory:
    def __init__(self, x=0, y=0, theta=0):
        """
        初始化小车状态
        :param x: 初始x坐标
        :param y: 初始y坐标
        :param theta: 初始朝向角度(弧度)
        """
        self.x = x
        self.y = y
        self.theta = theta  # 朝向角度，弧度制
        self.trajectory = [(x, y)]  # 存储轨迹点
        self.orientation = [0]  # 初始朝向角度(弧度)

        self.D = 0.3  # 小车前进的距离单位
        self.L = 0.702  # 小车的长度单位
        self.W = 0.610  # 小车的宽度单位
        self.d = 0.3  # 离墙距离
        self.k = 0.9  # 最大角度比例系数
        self.minimum_distance = 0.07  # 最小距离
        self.maximum_distance = 1.5  # 最大距离
        self.angle2wall = 0
        
    def rotate(self, angle_degrees):
        """
        旋转小车
        :param angle_degrees: 旋转角度(度)
        """
        angle_rad = np.radians(angle_degrees)
        self.theta += angle_rad
        # 规范化角度到[-π, π]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        self.orientation.append(self.theta)  # 保存朝向角度
        
    def move_forward(self, distance):
        """
        小车前进固定距离
        :param distance: 前进距离
        """
        dx = distance * np.cos(self.theta)
        dy = distance * np.sin(self.theta)
        self.x += dx
        self.y += dy
        self.trajectory.append((self.x, self.y))
        
    def get_position(self):
        """获取当前坐标"""
        return self.x, self.y
    
    def get_orientation(self):
        """获取当前朝向(度)"""
        return np.degrees(self.theta)
    
    def solve_theta(self):
        valid_solutions = []
        # 计算系数 A, B, C
        A = self.W**2 + 4 * (self.D + self.L/2)**2
        B = 2 * self.W * (self.d/2) - 4 * (self.D + self.L/2)**2
        C = (self.d/2)**2
        
        # 计算判别式
        discriminant = B**2 - 4 * A * C
        
        # 检查判别式非负
        if discriminant < 0:
            print("无实数解：判别式 B²-4AC < 0")
            return valid_solutions
        
        # 计算两个可能的解
        sqrt_discriminant = np.sqrt(discriminant)
        y1 = (-B + sqrt_discriminant) / (2 * A)
        y2 = (-B - sqrt_discriminant) / (2 * A)
        
        # 检查 y 是否在 [0,1] 范围内
        valid_solutions = []
        for y in [y1, y2]:
            if 0 <= y <= 1:
                x = np.sqrt(y)
                theta = 2 * np.arcsin(x)
                if 0 <= theta <= np.pi/2:  # 检查 θ 是否在 [0,π/2] 范围内
                    valid_solutions.append(theta*self.k)
        
        if not valid_solutions:
            print("无有效解：所有解均不在合理范围内")
            return valid_solutions
        
        return valid_solutions


if __name__ == "__main__":
    car = CarTrajectory(x=0,y=-0.7, theta=0)