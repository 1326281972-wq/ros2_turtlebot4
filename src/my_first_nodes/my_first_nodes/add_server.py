import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('服务端就绪：等待 add_two_ints 请求 ...')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('收到 %d + %d，返回 %d' % (request.a, request.b, response.sum))
        return response          # ← 必须 return response，这就是“响应”


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    