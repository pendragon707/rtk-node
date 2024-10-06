from req_res_str_service.srv import ReqRes

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(ReqRes, 'cam_service', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.res = "OK"
        self.get_logger().info(f"{request.req}")

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    