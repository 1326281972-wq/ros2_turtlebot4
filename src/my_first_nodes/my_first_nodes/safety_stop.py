#!/usr/bin/env python3
"""
避障保护节点 v2（入门任务 1 交付核心）
---------------------------------
作用：位于遥控器和底盘之间的"安全中间层"
  遥控器 ──>/teleop_cmd_vel──> 【本节点】 ──>/cmd_vel──> 底盘
                                   ↑
                                 /scan（雷达）

v2 三大改进（相对 v1 的"只会在 0.3m 急刹"）：
  1. 分级限速：0.6 m 外全速；0.3~0.6 m 内按距离比例降速（平滑，不急刹）
  2. 迟滞(hysteresis)：0.30 m 进入停车，0.40 m 才恢复 —— 防止临界点走走停停
  3. 脱困放行：被挡时只封锁"前进"，放行倒车和原地转向 —— 不用人工拉车

关键知识点：
  - 雷达是传感器数据，必须用 best_effort QoS(qos_profile_sensor_data)，
    用默认 reliable 订阅会一条数据都收不到（已实测验证）
  - 距离里要过滤 inf / nan（超量程和无效点）
  - /scan 实测只有 2 Hz（0.5s 一帧），所以必须持续发布、且阈值要留余量
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


STOP_DISTANCE = 0.30         # 进入停车状态的距离（米）
RESUME_DISTANCE = 0.40       # 退出停车状态的距离（迟滞，> STOP_DISTANCE）
SLOW_DISTANCE = 0.60         # 开始减速的距离（米）
FRONT_HALF_ANGLE = math.pi / 3   # 前方扇区半角：±60°
PUBLISH_RATE = 10.0          # 持续发布频率（Hz）


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
        self.blocked = False               # 停车状态（带迟滞）

        # ④ 定时发布：保证任何时刻都有输出（被挡住时持续发零速）
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        self.get_logger().info(
            f'避障节点 v2 启动：停车<{STOP_DISTANCE}m / 恢复>{RESUME_DISTANCE}m / '
            f'减速<{SLOW_DISTANCE}m，扇区 ±{int(math.degrees(FRONT_HALF_ANGLE))}°')

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
            # 该点对应的角度，换算到 [-pi, pi]，判断是否在正前方扇区内
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= FRONT_HALF_ANGLE:
                min_dist = min(min_dist, r)

        self.min_distance = min_dist

        # 迟滞状态机：进入容易退出难，避免 0.3m 附近抖动
        if self.blocked:
            if min_dist > RESUME_DISTANCE:
                self.blocked = False
                self.get_logger().info(
                    f'✓ 障碍已远离（{min_dist:.3f} m），恢复响应')
        else:
            if min_dist < STOP_DISTANCE:
                self.blocked = True
                self.get_logger().warn(
                    f'⚠ 前方障碍 {min_dist:.3f} m < {STOP_DISTANCE} m —— 停车'
                    f'（可倒车/转向脱困）')

    def timer_callback(self):
        out = Twist()

        if self.blocked:
            # 脱困放行：倒车(linear.x<0)和转向(angular)允许通过，前进一律清零
            if self.last_cmd.linear.x <= 0.0:
                out = self.last_cmd
            # 前进指令 → out 保持全零（持续发布，不是只发一次）
        else:
            out = self.last_cmd
            # 分级限速：越近越慢（只限制前进速度，倒车不限）
            if out.linear.x > 0.0 and self.min_distance < SLOW_DISTANCE:
                span = SLOW_DISTANCE - STOP_DISTANCE
                factor = max(0.0, (self.min_distance - STOP_DISTANCE) / span)
                out.linear.x *= factor

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
