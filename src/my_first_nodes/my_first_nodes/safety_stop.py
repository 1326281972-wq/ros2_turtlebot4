#!/usr/bin/env python3
"""
避障保护节点 v3.1 —— 碰撞预测 + 速度限幅 + Fail-Safe（入门任务 1 交付核心）
=========================================================================
  遥控器 ──>/teleop_cmd_vel──> 【本节点】 ──>/cmd_vel──> 底盘
                                   ↑
                                 /scan（雷达）

v3.1 相对 v3 修复的三个致命问题（都是真机实测撞墙后定位的）
----------------------------------------------------------
【致命 BUG】近距盲区：RPLIDAR 的 range_min = 0.15 m，v3 把所有 r < range_min 的点
    当作"无效"丢弃。当车蹭到 0.15 m 以内时，前方所有点都变无效 → 最小距离变成 inf
    → 允许速度变成无穷大 → 全速撞上去。
    修复：0 < r < range_min 时按 range_min 计入（"贴脸"仍算有障碍）。

【Fail-Safe 1】雷达数据超时：超过 SCAN_TIMEOUT 秒没收到 /scan（雷达掉线、话题中断），
    一律把允许速度压到 0 —— 安全系统必须在"看不见"时选择停车，而不是全速。

【Fail-Safe 2】空帧保护：整帧没有任何有效点（如全被遮挡/超量程）时，
    保留上一帧的距离，不把距离放大成 inf。

物理常识（真机必知，答辩加分）
--------------------------
- 2D 雷达只扫一层水平面：低矮物体（< 雷达高度）、黑色吸光、玻璃/镜面 都可能"看不见"，
  这类场景只能靠底盘自带 bump/cliff 传感器兜底，不是软件能解决的。
- /scan 实测 2 Hz → 反应时间必须 ≥ 0.5 s，否则限速公式给的允许速度偏大。

关键知识点：
  - 传感器数据必须 best_effort QoS(qos_profile_sensor_data)，默认 reliable 收不到
  - 距离要过滤 inf / nan，但**不能**把"太近"当成"没有"
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ===== 可调参数 =====
HARD_STOP_DISTANCE = 0.12    # 紧急停车距离（米）
SAFETY_MARGIN = 0.10         # 安全余量（米）：雷达安装点到车头前缘
REACTION_TIME = 0.70         # 反应时间（秒）= 雷达周期 0.5 + 通信制动 0.2
SCAN_TIMEOUT = 1.5           # 雷达数据超时（秒）→ 触发 fail-safe 停车
FRONT_HALF_ANGLE = math.pi / 3   # 前方扇区半角 ±60°
PUBLISH_RATE = 20.0          # 控制频率（Hz）
LOG_PERIOD = 1.0             # 日志限频（秒）


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sub_cmd = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.min_distance = float('inf')
        self.range_min = 0.15          # 收到第一帧后由 msg.range_min 更新
        self.last_scan_time = None     # 最近一次收到 scan 的时刻（秒）
        self.last_cmd = Twist()
        self.last_log_time = 0.0

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        self.get_logger().info(
            f'避障 v3.1 启动：急停<{HARD_STOP_DISTANCE}m 余量{SAFETY_MARGIN}m '
            f'反应{REACTION_TIME}s 数据超时{SCAN_TIMEOUT}s '
            f'扇区±{int(math.degrees(FRONT_HALF_ANGLE))}°')

    # ---------------- 回调 ----------------
    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def scan_callback(self, msg: LaserScan):
        self.range_min = msg.range_min
        self.last_scan_time = self.get_clock().now().nanoseconds / 1e9

        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            if r is None:
                continue
            if math.isinf(r) or math.isnan(r):        # 超量程/测量失败
                continue
            if r <= 0.0:                              # 无效
                continue
            # ★ 关键修复：比 range_min 还近 → 说明"贴脸"，按 range_min 计入（不能丢弃！）
            r_eff = max(r, msg.range_min)
            if r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= FRONT_HALF_ANGLE:
                min_dist = min(min_dist, r_eff)

        # ★ Fail-Safe 2：整帧无有效点 → 保留上一帧距离，绝不放大成 inf
        if math.isinf(min_dist):
            if math.isinf(self.min_distance):
                # 从未测到过任何东西，用超量程值代替（相当于"前方空旷"）
                self.min_distance = msg.range_max
        else:
            self.min_distance = min_dist

        self.get_logger().info(
            f'雷达帧：前方最近 {self.min_distance:.3f} m（有效点中最近角度已过滤）')

    # ---------------- 核心逻辑 ----------------
    def allowed_linear_speed(self, distance: float) -> float:
        if distance < HARD_STOP_DISTANCE:
            return 0.0
        return max(0.0, (distance - SAFETY_MARGIN) / REACTION_TIME)

    def timer_callback(self):
        out = Twist()
        out.angular = self.last_cmd.angular          # ★ 转向永远放行
        out.linear.y = self.last_cmd.linear.y

        v_want = self.last_cmd.linear.x
        now = self.get_clock().now().nanoseconds / 1e9

        # ★ Fail-Safe 1：雷达数据超时 → 禁止前进
        scan_stale = (self.last_scan_time is None
                      or (now - self.last_scan_time) > SCAN_TIMEOUT)

        if v_want <= 0.0:
            out.linear.x = v_want                    # 倒车/零速永远放行
        elif scan_stale:
            out.linear.x = 0.0
            if now - self.last_log_time > LOG_PERIOD:
                self.last_log_time = now
                self.get_logger().error(
                    f'❌ 雷达数据超时（>{SCAN_TIMEOUT}s 未收到 /scan）→ 禁止前进')
        else:
            v_allow = self.allowed_linear_speed(self.min_distance)
            out.linear.x = min(v_want, v_allow)
            if out.linear.x < v_want - 1e-3 and now - self.last_log_time > LOG_PERIOD:
                self.last_log_time = now
                if v_allow <= 0.0:
                    self.get_logger().warn(
                        f'🛑 紧急停车：前方 {self.min_distance:.3f} m < {HARD_STOP_DISTANCE} m')
                else:
                    self.get_logger().warn(
                        f'减速保护：前方 {self.min_distance:.3f} m → '
                        f'{v_want:.2f} 限到 {out.linear.x:.2f} m/s')

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
