import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from turtlesim.msg import Pose
from geometry_msgs.msg import PointStamped


class TurtleToPoint(Node):

    def __init__(self):
        super().__init__('turtle_to_point')
        self.path_pub = self.create_publisher(Path, '/turtle_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(PointStamped, '/turtle_point', 10)
        self.get_logger().info('turtle_to_point 就绪：/turtle1/pose -> /turtle_point')

    def pose_callback(self, msg):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = msg.x
        ps.pose.position.y = msg.y
        ps.pose.orientation.w = 1.0
        self.path.poses.append(ps)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()   # 时间戳（必须有）
        p.header.frame_id = 'map'                          # 坐标系（必须有）
        p.point.x = msg.x
        p.point.y = msg.y
        p.point.z = 0.0
        self.publisher_.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()