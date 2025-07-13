import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("[INIT] TestNode initialized")

    def timer_callback(self):
        self.get_logger().info("[TIMER] Timer callback executed")

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node.get_logger().debug("[MAIN] Spin once")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()