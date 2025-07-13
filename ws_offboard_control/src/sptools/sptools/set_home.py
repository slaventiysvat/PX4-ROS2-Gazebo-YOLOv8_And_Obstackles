import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandHome

class SetHomeNode(Node):
    def __init__(self):
        super().__init__('set_home_node')
        self.cli = self.create_client(CommandHome, '/mavros/cmd/set_home')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Очікування сервісу set_home...')
        self.set_home()

    def set_home(self):
        req = CommandHome.Request()
        req.current_gps = False
        req.latitude = 47.396819
        req.longitude = 8.549716
        req.altitude = 3.86
        future = self.cli.call_async(req)
        future.add_done_callback(self.home_response)

    def home_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Точку походження успішно встановлено')
            else:
                self.get_logger().error(f'Не вдалося встановити точку походження: {response.result}')
        except Exception as e:
            self.get_logger().error(f'Помилка сервісу: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SetHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()