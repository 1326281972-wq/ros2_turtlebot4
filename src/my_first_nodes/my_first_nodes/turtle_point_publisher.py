import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import PointStamped


class TurtlePointPublisher(Node):

    def __init__(self):
        super().__init__('turtle_point_publisher')

        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(PointStamped, '/turtle_point', 10)

        self.get_logger().info('turtle_point 已启动：/turtle1/pose -> /turtle_point')

    def pose_callback(self, msg):
        point = PointStamped()

        # rviz2 能不能显示，全看这两个字段
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'map'      # 必须写，否则 rviz2 不知道往哪个坐标系摆

        point.point.x = msg.x              # turtlesim 的 x：0~11.08
        point.point.y = msg.y              # turtlesim 的 y：0~11.08
        point.point.z = 0.0                # 2D，平铺 z=0

        self.publisher_.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
