import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist
import time

class ArmingNode(Node):
    def __init__(self):
        super().__init__('arming_node')

        # Паблішер для setpoint_velocity (для підтримки OFFBOARD)
        self.cmd_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Клієнт для армування
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Очікуємо /mavros/cmd/arming сервіс...')

        # Клієнт для зміни режиму
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Очікуємо /mavros/set_mode сервіс...')

        # Таймер для регулярної публікації пустих Twist (20 Гц)
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

        # Виконуємо армування через 2 секунди
        self.create_timer(2.0, self.arm_and_set_mode)

    def publish_cmd_vel(self):
        twist = Twist()
        self.cmd_publisher.publish(twist)
        self.get_logger().debug('Опубліковано нульову швидкість для OFFBOARD')

    def arm_and_set_mode(self):
        # Змінити режим на OFFBOARD
        req_mode = SetMode.Request()
        req_mode.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req_mode)
        future.add_done_callback(self.set_mode_callback)
        self.get_logger().info('Встановлюємо режим OFFBOARD')

        # Армувати
        time.sleep(0.5)  # Дати час на встановлення режиму
        req_arm = CommandBool.Request()
        req_arm.value = True
        future = self.arming_client.call_async(req_arm)
        future.add_done_callback(self.arming_callback)
        self.get_logger().info('Відправляємо команду ARM')

    def set_mode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('OFFBOARD режим успішно встановлено')
            else:
                self.get_logger().warn('Не вдалося встановити OFFBOARD режим')
        except Exception as e:
            self.get_logger().error(f'Помилка встановлення режиму: {e}')

    def arming_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Озброєння успішне')
            else:
                self.get_logger().warn(f'Озброєння не вдалося: {response.result}')
        except Exception as e:
            self.get_logger().error(f'Помилка озброєння: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()