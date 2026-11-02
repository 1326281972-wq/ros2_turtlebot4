#!/usr/bin/env python3
"""
避障保护节点 v3.2 —— 碰撞预测 + 速度限幅 + QoS 自适应 + Fail-Safe
=================================================================
  遥控器 ──>/teleop_cmd_vel──> 【本节点】 ──>/cmd_vel──> 底盘
                                   ↑
                                 /scan（雷达）

v3.2 相对 v3.1 的两个关键修复
----------------------------
【致命，实测定位】QoS 不兼容：本机 `/scan` 的发布者 `rplidar_composition` 使用
    **RELIABLE**，而 v3.1 用 `qos_profile_sensor_data`（BEST_EFFORT）订阅。
    按 DDS 兼容规则「发布者 RELIABLE 时，订阅者 BEST_EFFORT 不兼容」→ 一条数据都收不到
    → 节点一直报"雷达数据超时"。
    修复：同时建立 RELIABLE 与 BEST_EFFORT 两个订阅（自动适配），
          无论雷达节点把 QoS 配成哪种都能收到。工业上叫 QoS 自适应。

          QoS 兼容矩阵：
                      订阅者 RELIABLE | 订阅者 BEST_EFFORT
            发布 RELIABLE     ✅        |      ❌
            发布 BEST_EFFORT  ❌        |      ✅

【体验】日志只在"状态变化"时打印（进入限速/进入急停/恢复正常/数据恢复各一次），
    不再每秒刷屏。

保留的 fail-safe
---------------
1. 近距盲区：0 < r < range_min 时按 range_min 计入（贴脸也算有障碍）
2. 数据超时：>SCAN_TIMEOUT 秒没数据 → 禁止前进（看不见就停车）
3. 空帧保护：整帧无有效点时保留上一帧距离，绝不放大成 inf

设计理念（可写进报告）
-------------------
允许最大前进速度 = (前方距离 − 安全余量) / 反应时间
实际输出速度     = min(遥控期望速度, 允许最大前进速度)
转向/倒车永远放行 → 狭小空间可以边转边蹭，不会被困死。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ===== 可调参数 =====
HARD_STOP_DISTANCE = 0.12    # 紧急停车距离（米）
SAFETY_MARGIN = 0.10         # 安全余量（米）：雷达安装点到车头前缘
REACTION_TIME = 0.70         # 反应时间（秒）= 雷达周期 0.5 + 通信制动 0.2
SCAN_TIMEOUT = 1.5           # 雷达数据超时（秒）→ fail-safe 禁止前进
FRONT_HALF_ANGLE = math.pi / 3   # 前方扇区半角 ±60°
PUBLISH_RATE = 20.0          # 控制频率（Hz）
LOG_FRAME_EVERY = 10         # 每 N 帧打印一次前方距离（2Hz 雷达约 5 秒一次）

# 两种 QoS 都挂上，雷达节点用哪种都能收到
QOS_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # ★ QoS 自适应：同一话题建两个订阅，谁匹配上用谁
        self.sub_scan_reliable = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, QOS_RELIABLE)
        self.sub_scan_besteffort = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.sub_cmd = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.min_distance = float('inf')
        self.last_scan_time = None
        self.last_cmd = Twist()

        # 日志用状态位（只在变化时打印，避免刷屏）
        self.last_zone = 'INIT'      # INIT / SAFE / SLOW / STOP
        self.scan_ok = None          # None=未知, True=正常, False=超时
        self.frame_count = 0

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        self.get_logger().info(
            f'避障 v3.2(QoS自适应) 启动：急停<{HARD_STOP_DISTANCE}m '
            f'余量{SAFETY_MARGIN}m 反应{REACTION_TIME}s 超时{SCAN_TIMEOUT}s '
            f'扇区±{int(math.degrees(FRONT_HALF_ANGLE))}°')

    # ---------------- 回调 ----------------
    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now().nanoseconds / 1e9

        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            if r is None:
                continue
            if math.isinf(r) or math.isnan(r):        # 超量程/测量失败
                continue
            if r <= 0.0:
                continue
            if r > msg.range_max:
                continue
            # ★ 近距盲区修复：比 range_min 还近 → 按 range_min 计入（不能丢！）
            r_eff = max(r, msg.range_min)

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= FRONT_HALF_ANGLE:
                min_dist = min(min_dist, r_eff)

        # ★ 空帧保护：无有效点时保留上一帧距离，绝不改成 inf
        if math.isinf(min_dist):
            if math.isinf(self.min_distance):
                self.min_distance = msg.range_max
        else:
            self.min_distance = min_dist

        self.frame_count += 1
        if self.frame_count % LOG_FRAME_EVERY == 1:
            self.get_logger().info(
                f'雷达帧 #{self.frame_count}：前方最近 {self.min_distance:.3f} m')

    # ---------------- 核心逻辑 ----------------
    def allowed_linear_speed(self, distance: float) -> float:
        if distance < HARD_STOP_DISTANCE:
            return 0.0
        return max(0.0, (distance - SAFETY_MARGIN) / REACTION_TIME)

    def zone_of(self, distance: float) -> str:
        if distance < HARD_STOP_DISTANCE:
            return 'STOP'
        if distance < SAFETY_MARGIN + REACTION_TIME * 0.05:   # 约 0.135 m 以上但很近
            return 'SLOW'
        return 'SAFE'

    def log_transition(self, zone: str, v_want: float, v_out: float):
        """只在安全区间发生变化时打印一条（不刷屏）"""
        if zone == self.last_zone:
            return
        self.last_zone = zone
        if zone == 'STOP':
            self.get_logger().warn(
                f'🛑 紧急停车：前方 {self.min_distance:.3f} m < {HARD_STOP_DISTANCE} m'
                f'（转向/倒车仍放行）')
        elif zone == 'SLOW':
            self.get_logger().warn(
                f'⚠ 减速保护：前方 {self.min_distance:.3f} m → '
                f'{v_want:.2f} 限到 {v_out:.2f} m/s')
        else:
            self.get_logger().info(
                f'✓ 前方安全（{self.min_distance:.3f} m），全速放行')

    def timer_callback(self):
        out = Twist()
        out.angular = self.last_cmd.angular          # ★ 转向永远放行
        out.linear.y = self.last_cmd.linear.y
        v_want = self.last_cmd.linear.x

        now = self.get_clock().now().nanoseconds / 1e9
        stale = (self.last_scan_time is None
                 or (now - self.last_scan_time) > SCAN_TIMEOUT)

        # ★ 数据超时：只在状态切换时打印一次
        if stale != (self.scan_ok is False):
            if stale:
                self.scan_ok = False
                self.get_logger().error(
                    f'❌ 雷达数据超时（>{SCAN_TIMEOUT}s 无 /scan）→ 禁止前进'
                    f'（检查 /scan 是否在发布、QoS 是否兼容）')
            else:
                self.scan_ok = True
                self.get_logger().info('✓ 雷达数据恢复，恢复正常控制')

        if v_want <= 0.0:
            out.linear.x = v_want                    # 倒车/零速永远放行
        elif stale:
            out.linear.x = 0.0                       # 看不见 → 不准前进
        else:
            v_allow = self.allowed_linear_speed(self.min_distance)
            out.linear.x = min(v_want, v_allow)
            self.log_transition(self.zone_of(self.min_distance), v_want, out.linear.x)

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
