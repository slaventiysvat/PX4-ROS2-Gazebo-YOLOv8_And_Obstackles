# save as minimal_takeoff_node.py inside your sptools package
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import math
import time

class MinimalTakeoff(Node):
    def __init__(self):
        super().__init__('minimal_takeoff')
        self.state = State()
        self.pose = PoseStamped()

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.ready = False
        self.takeoff_complete = False
        self.finished = False
        self.get_logger().info("Node initialized")

    def state_callback(self, msg):
        self.state = msg
        if msg.connected:
            self.ready = True

    def timer_callback(self):
        if not self.ready:
            self.get_logger().info("Waiting for FCU...")
            return
        if self.finished:
            return
        if self.state == 'LANDING_COMPLETE':
            self.get_logger().info("Mission complete. Stopping timer.")
            self.finished = True
            self.timer.cancel()
            return

        if not self.state.armed or self.state.mode != "OFFBOARD":
            # Initial pose
            self.pose.pose.position.z = 2.5
            self.pose.pose.orientation.w = 1.0
            self.pose.header.stamp = self.get_clock().now().to_msg()

            for _ in range(20):
                self.setpoint_pub.publish(self.pose)
                time.sleep(0.05)

            # Set OFFBOARD
            mode_req = SetMode.Request()
            mode_req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(mode_req)
            self.get_logger().info("OFFBOARD requested")

            # Arm
            arm_req = CommandBool.Request()
            arm_req.value = True
            self.arming_client.call_async(arm_req)
            self.get_logger().info("Arming requested")
            return

        if not self.takeoff_complete:
            self.get_logger().info("Taking off and rotating...")
            for i in range(10):  # Rotate 360 in 10 steps
                yaw = math.radians(i * 36)
                self.pose.pose.orientation.z = math.sin(yaw / 2.0)
                self.pose.pose.orientation.w = math.cos(yaw / 2.0)
                self.pose.header.stamp = self.get_clock().now().to_msg()
                self.setpoint_pub.publish(self.pose)
                time.sleep(0.5)
            self.takeoff_complete = True
            self.get_logger().info("Rotation complete, landing...")
            self.pose.pose.position.z = 0.0
            for _ in range(40):
                self.setpoint_pub.publish(self.pose)
                time.sleep(0.1)
            self.get_logger().info("Landing complete")
            self.get_logger().info("Mission complete. Stopping timer.")
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = MinimalTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
