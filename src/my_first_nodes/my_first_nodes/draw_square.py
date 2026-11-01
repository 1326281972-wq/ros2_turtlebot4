import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rcl_interfaces.msg import SetParametersResult


class DrawSquare(Node):

    def __init__(self):
        super().__init__('draw_square')

        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('dist_tol', 0.02)
        self.declare_parameter('angle_tol', 0.03)

        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.dist_tol = self.get_parameter('dist_tol').value
        self.angle_tol = self.get_parameter('angle_tol').value

        # 参数被外部修改时自动触发
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.pose = None            # 还没有收到任何位置数据

        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)   # 20 Hz 控制回路

        self.state = 'START'
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_yaw = 0.0
        self.edges_done = 0

        self.get_logger().info('draw_square 已启动，等待 /turtle1/pose ...')

    def parameter_callback(self, params):
        for p in params:
            self.get_logger().info('参数 %s 被改为 %s' % (p.name, p.value))
            if p.name == 'side_length':
                self.side_length = p.value
            elif p.name == 'linear_speed':
                self.linear_speed = p.value
            elif p.name == 'angular_speed':
                self.angular_speed = p.value
            elif p.name == 'dist_tol':
                self.dist_tol = p.value
            elif p.name == 'angle_tol':
                self.angle_tol = p.value
        return SetParametersResult(successful=True)

    def pose_callback(self, msg):
        self.pose = msg

    @staticmethod
    def normalize(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        cmd = Twist()                       # 默认全 0，等于“不动”

        if self.pose is None:               # 还没拿到位置，什么都别做
            return

        if self.state == 'START':
            self.start_x = self.pose.x
            self.start_y = self.pose.y
            self.target_yaw = self.pose.theta
            self.get_logger().info('第 %d 条边：出发' % (self.edges_done + 1))
            self.state = 'FORWARD'

        elif self.state == 'FORWARD':
            dist = math.hypot(self.pose.x - self.start_x, self.pose.y - self.start_y)
            if dist < self.side_length - self.dist_tol:
                cmd.linear.x = self.linear_speed
            else:
                self.target_yaw = self.normalize(self.pose.theta + math.pi / 2.0)
                self.get_logger().info('第 %d 条边完成 -> 转向 90 度' % (self.edges_done + 1))
                self.state = 'TURN'

        elif self.state == 'TURN':
            err = self.normalize(self.target_yaw - self.pose.theta)
            if abs(err) > self.angle_tol:
                cmd.angular.z = self.angular_speed if err > 0 else -self.angular_speed
            else:
                self.edges_done += 1
                if self.edges_done >= 4:
                    self.get_logger().info('正方形完成！共 4 条边')
                    self.state = 'DONE'
                else:
                    self.state = 'START'

        elif self.state == 'DONE':
            pass                            # cmd 保持全 0 => 停住

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()