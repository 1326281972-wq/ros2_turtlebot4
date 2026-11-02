#!/usr/bin/env python3
"""
避障保护节点（入门任务 1 交付核心）
---------------------------------
作用：位于遥控器和底盘之间的"安全中间层"
  遥控器 ──>/teleop_cmd_vel──> 【本节点】 ──>/cmd_vel──> 底盘
                                   ↑
                                 /scan（雷达）

规则：
  1. 前方扇区内最近障碍 < SAFE_DISTANCE(0.3 m) → 忽略遥控指令，持续发布零速并打印距离
  2. 否则原样转发遥控指令
  3. 未收到遥控指令时也不停发零速，保证"被挡住期间"底盘始终有 0 速输入

关键知识点：
  - 雷达是传感器数据，必须用 best_effort QoS(qos_profile_sensor_data)，
    用默认 reliable 订阅会一条数据都收不到（已实测验证）
  - 距离里要过滤 inf / nan（超量程和无效点）
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


SAFE_DISTANCE = 0.3          # 停车阈值（米）
FRONT_HALF_ANGLE = math.pi / 3   # 前方扇区半角：±60°
PUBLISH_RATE = 10.0          # 持续发布频率（Hz），保证挡住时不断发零速


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # ① 订阅雷达：必须用传感器 QoS（best_effort）
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ② 订阅遥控器（已改道到 /teleop_cmd_vel）
        self.sub_cmd = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10)

        # ③ 发布真正的底盘指令
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.min_distance = float('inf')   # 前方最近障碍距离
        self.last_cmd = Twist()            # 遥控最新指令
        self.blocked = False

        # ④ 定时发布：保证任何时刻都有输出（被挡住时持续发零速）
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        self.get_logger().info(
            f'避障节点启动：阈值 {SAFE_DISTANCE} m，前方扇区 ±{int(math.degrees(FRONT_HALF_ANGLE))}°')

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def scan_callback(self, msg: LaserScan):
        """只统计前方扇区内的有效距离"""
        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            if r is None:
                continue
            # 过滤无效点（inf：超量程；nan：测量失败）
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            # 该点对应的角度
            angle = msg.angle_min + i * msg.angle_increment
            # 换算到 [-pi, pi]，判断是否在正前方扇区内
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= FRONT_HALF_ANGLE:
                min_dist = min(min_dist, r)

        self.min_distance = min_dist
        self.blocked = min_dist < SAFE_DISTANCE

        if self.blocked:
            self.get_logger().warn(
                f'⚠ 前方障碍 {min_dist:.3f} m < {SAFE_DISTANCE} m —— 已停车')

    def timer_callback(self):
        out = Twist()
        if self.blocked:
            out = Twist()          # 全零 = 停车（持续发送，不是只发一次）
        else:
            out = self.last_cmd
        self.pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
