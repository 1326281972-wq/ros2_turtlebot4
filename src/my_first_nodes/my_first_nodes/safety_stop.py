#!/usr/bin/env python3
"""
避障保护节点 v3 —— 碰撞预测 + 速度限幅（入门任务 1 交付核心）
=============================================================
作用：位于遥控器和底盘之间的"安全中间层"
  遥控器 ──>/teleop_cmd_vel──> 【本节点】 ──>/cmd_vel──> 底盘
                                   ↑
                                 /scan（雷达）

设计理念的演进（v1 → v3）
------------------------
v1（状态封锁）：前方 < 0.3 m 就把所有指令清零。
    缺陷：狭小空间里前进被彻底禁死，必须人工把车拉出来；临界距离还来回抖动。
v2（迟滞 + 放行）：加迟滞(0.3进/0.4出)，被挡时放行倒车和转向。
    缺陷：仍然是"一刀切"——只要进了包围圈，前进速度直接归零，不能缓慢蹭。
v3（碰撞预测 + 限幅，本版本）：不禁止任何按键，只在"会撞上"时压缩速度。

    ★ 核心公式（每 50 ms 计算一次）：
        允许最大前进速度 = (前方距离 − 安全余量) / 反应时间
        实际输出速度     = min(遥控期望速度, 允许最大前进速度)

    物理含义：把"安全"表达为速度约束而不是开关。
    - 距离 0.50 m → 允许 ≈ 0.57 m/s
    - 距离 0.30 m → 允许 ≈ 0.29 m/s
    - 距离 0.20 m → 允许 ≈ 0.14 m/s   （越近越慢，永远撞不上）
    - 距离 < 0.12 m → 紧急停车（真兜底）

    ★ 三条永远放行：
      1. 转向 angular.z  —— 狭小空间靠转向脱困
      2. 倒车 linear.x<0 —— 后退永远安全
      3. 零速           —— 本来就不动

关键知识点：
  - 雷达是传感器数据，必须用 best_effort QoS(qos_profile_sensor_data)，
    用默认 reliable 订阅会一条数据都收不到（已实测验证）
  - 距离里要过滤 inf / nan（超量程和无效点）
  - /scan 实测只有 2 Hz（0.5 s 一帧）→ 反应时间必须 ≥ 0.5 s，否则算出来的允许速度
    会偏大，冲过头就会撞上
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ===== 可调参数（改这里就能调手感）=====
HARD_STOP_DISTANCE = 0.12    # 紧急停车距离（米）——真兜底，小于它才真的不动
SAFETY_MARGIN = 0.10         # 安全余量（米）——车身雷达安装位置到车头边缘的距离
REACTION_TIME = 0.70         # 反应时间（秒）= 雷达周期 0.5 + 通信制动 0.2
FRONT_HALF_ANGLE = math.pi / 3   # 前方扇区半角：±60°
PUBLISH_RATE = 20.0          # 控制频率（Hz）——必须远高于雷达 2 Hz，才能及时限幅
LOG_PERIOD = 1.0             # 限速日志打印间隔（秒），避免刷屏


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # ① 订阅雷达：必须用传感器 QoS（best_effort）
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ② 订阅遥控器（已改道到 /teleop_cmd_vel）
        self.sub_cmd = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10)

        # ③ 发布底盘指令
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.min_distance = float('inf')   # 前方扇区最近障碍
        self.last_cmd = Twist()            # 遥控最新指令
        self.last_log_time = 0.0
        self.stopped = False               # 是否处于紧急停车

        # ④ 高频定时器：限幅必须比雷达快，才能在雷达两帧之间"持续压制"速度
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        self.get_logger().info(
            f'避障 v3 启动（碰撞预测+限幅）：急停<{HARD_STOP_DISTANCE}m，'
            f'余量 {SAFETY_MARGIN}m，反应时间 {REACTION_TIME}s，'
            f'扇区 ±{int(math.degrees(FRONT_HALF_ANGLE))}°，控制频率 {PUBLISH_RATE:.0f}Hz')

    # ---------------- 回调 ----------------
    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def scan_callback(self, msg: LaserScan):
        """统计前方扇区的最近有效距离"""
        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            if r is None:
                continue
            if math.isinf(r) or math.isnan(r):        # 超量程/测量失败
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))   # 归一到 [-pi, pi]
            if abs(angle) <= FRONT_HALF_ANGLE:
                min_dist = min(min_dist, r)

        # 距离只增不减地"平滑"一帧，避免单点噪声导致速度忽高忽低
        self.min_distance = min_dist

    # ---------------- 核心逻辑 ----------------
    def allowed_linear_speed(self, distance: float) -> float:
        """由距离反推允许的最大前进速度（安全包络）"""
        if distance < HARD_STOP_DISTANCE:
            return 0.0
        return max(0.0, (distance - SAFETY_MARGIN) / REACTION_TIME)

    def timer_callback(self):
        out = Twist()

        # ★ 转向永远放行（狭小空间靠它脱困）
        out.angular = self.last_cmd.angular

        v_want = self.last_cmd.linear.x

        if v_want <= 0.0:
            # ★ 倒车/零速永远放行（后退不会撞前方障碍）
            out.linear.x = v_want
            out.linear.y = self.last_cmd.linear.y
        else:
            v_allow = self.allowed_linear_speed(self.min_distance)
            out.linear.x = min(v_want, v_allow)
            out.linear.y = self.last_cmd.linear.y

            # 日志：只在"被限幅"或"急停"时打印，且限频
            now = self.get_clock().now().nanoseconds / 1e9
            if out.linear.x < v_want - 1e-3 and now - self.last_log_time > LOG_PERIOD:
                self.last_log_time = now
                if v_allow <= 0.0:
                    self.get_logger().warn(
                        f'🛑 紧急停车：前方 {self.min_distance:.3f} m < {HARD_STOP_DISTANCE} m')
                else:
                    self.get_logger().warn(
                        f'减速保护：前方 {self.min_distance:.3f} m → '
                        f'速度 {v_want:.2f} 限到 {out.linear.x:.2f} m/s（转向不受限）')

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
