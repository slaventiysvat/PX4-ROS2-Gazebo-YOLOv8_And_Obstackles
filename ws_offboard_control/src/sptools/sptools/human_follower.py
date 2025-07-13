# FSM версія human_follower.py (WAITING_FOR_FCU → ARMING → TAKING_OFF → MOVING_TO_PATCH → ROTATING → SCANNING → APPROACHING → RETURNING → LANDING → DONE → RECHECK_ROTATION → POST_APPROACH_ROTATE)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math
import cv2
from cv_bridge import CvBridge
from sptools.vision import detect_humans
import numpy as np
import struct
from sklearn.cluster import DBSCAN
from ultralytics import YOLO
from sptools.utils import pid_controller

class HumanFollowerFSM(Node):
    class StateEnum:
        WAITING_FOR_FCU = 0
        ARMING = 1
        TAKING_OFF = 2
        MOVING_TO_PATCH = 3
        ROTATING = 4
        SCANNING = 5
        APPROACHING = 6
        RETURNING = 7
        LANDING = 8
        DONE = 9
        RECHECK_ROTATION = 10
        POST_APPROACH_ROTATE = 11

    def __init__(self):
        super().__init__('human_follower')
        self.get_logger().info("[INIT] Node initialized")

        # Налаштування QoS профілів
        qos = QoSProfile(depth=10)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Підписки на теми
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos)
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_best_effort
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            qos_best_effort
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            qos_best_effort
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.depth_image_callback,
            qos_best_effort
        )

        # Публікатори
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_people_markers', qos)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/depth_pointcloud', qos)
        self.overlay_pub = self.create_publisher(Image, '/overlay_image', qos)
        self.depth_overlay_pub = self.create_publisher(Image, '/depth_overlay', qos)

        # Клієнти сервісів
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Налаштування трансформації
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # Таймер для періодичних викликів
        self.timer = self.create_timer(0.5, self.timer_callback)  # Зменшено частоту для YOLO

        # Ініціалізація пози
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.pose.position.z = 2.0
        self.pose.pose.orientation.w = 1.0

        # Змінні стану
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.latest_depth_image_msg = None
        self.camera_info = None
        self.fsm_state = self.StateEnum.WAITING_FOR_FCU
        self.takeoff_start = None
        self.rotate_start = None
        self.time_now = self.get_clock().now

        self.target_positions = []
        self.current_target_idx = 0
        self.first_target_position = None
        self.max_no_detection_cycles = 30
        self.no_detection_counter = 0
        self.detected_target_coords = []
        self.just_visited_position = None
        self.scanning_delay_counter = 0
        self.scanning_delay_max = 8
        self.total_people_detected = 0
        self.total_groups_detected = 0
        try:
            self.yolo_model = YOLO('yolov8n.pt').to("cuda")
            self.get_logger().info("[INIT] YOLO model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"[INIT] Failed to load YOLO model: {e}")
            raise

        # Патч-позиції (з висотою 2.0 м для сумісності з TAKEOFF)
        self.patch_positions = [
            Point(x=2.72, y=-2.80, z=2.0),
            Point(x=-4.78, y=7.65, z=2.0),
            Point(x=2.63, y=12.11, z=2.0),
            Point(x=-12.09, y=11.93, z=2.0),
            Point(x=16.98, y=12.20, z=2.0),
            Point(x=2.29, y=26.94, z=2.0),
            Point(x=16.59, y=27.00, z=2.0),
        ]
        self.current_patch_idx = 0

        # PID для плавного руху
        self.pid_gains = {'kp': 0.5, 'ki': 0.01, 'kd': 0.1}
        self.pid_error_sum = {'x': 0.0, 'y': 0.0}
        self.prev_error = {'x': 0.0, 'y': 0.0}

    def time_now(self):
        return self.get_clock().now().to_msg()

    def publish_pose(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = "map"
        self.setpoint_pub.publish(self.pose)

    def is_new_target(self, new_point: Point, threshold: float = 1.0) -> bool:
        for old_point in self.detected_target_coords:
            dx = new_point.x - old_point.x
            dy = new_point.y - old_point.y
            dz = new_point.z - old_point.z
            if math.sqrt(dx*dx + dy*dy + dz*dz) < threshold:
                return False
        return True

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def cluster_targets(self, targets):
        if not targets:
            return []
        points = [[t['point'].x, t['point'].y] for t in targets]
        clustering = DBSCAN(eps=1.0, min_samples=2).fit(points)
        labels = clustering.labels_
        clustered_targets = []
        for label in set(labels):
            if label != -1:
                cluster_points = [targets[i] for i in range(len(targets)) if labels[i] == label]
                centroid = Point(
                    x=sum(p['point'].x for p in cluster_points) / len(cluster_points),
                    y=sum(p['point'].y for p in cluster_points) / len(cluster_points),
                    z=2.0
                )
                clustered_targets.append({
                    'point': centroid,
                    'count': len(cluster_points)
                })
            else:
                for i in range(len(targets)):
                    if labels[i] == -1:
                        clustered_targets.append(targets[i])
        return clustered_targets

    def scanning_state_logic(self):
        if self.latest_image_msg is None or self.latest_depth_image_msg is None:
            self.get_logger().warn("[FSM] No image or depth image received yet.")
            return

        self.get_logger().info("[FSM] Scanning for humans...")
        current_position = self.current_pose.pose.position
        yaw_angle = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)

        targets = detect_humans(
            self.latest_image_msg,
            self.latest_depth_image_msg,
            self.bridge,
            current_position,
            yaw_angle,
            image_center=(960, 540),
            logger=self.get_logger(),
            model=self.yolo_model
        )

        filtered_targets = []
        for t in targets:
            p = t['point']
            is_duplicate = any(
                math.dist((p.x, p.y), (old.x, old.y)) < 0.5
                for old in self.detected_target_coords
            )
            if not is_duplicate:
                filtered_targets.append(t)
                self.detected_target_coords.append(p)

        filtered_targets = self.cluster_targets(filtered_targets)

        if filtered_targets:
            self.get_logger().info(f"[FSM] Detected {len(filtered_targets)} targets (groups or individuals).")
            self.total_people_detected += sum(t.get('count', 1) for t in filtered_targets)
            self.total_groups_detected += sum(1 for t in filtered_targets if t.get('count', 1) > 1)
            self.get_logger().info(f"[FSM] Total people: {self.total_people_detected}, Total groups: {self.total_groups_detected}")
            for i, target in enumerate(filtered_targets):
                self.get_logger().info(f"[FSM] Target #{i}: x={target['point'].x:.2f}, y={target['point'].y:.2f}, count={target.get('count', 1)}")
            self.target_positions = self.sort_targets_by_distance(filtered_targets, current_position)
            self.current_target_idx = 0
            if not self.first_target_position:
                self.first_target_position = Point(
                    x=self.current_pose.pose.position.x,
                    y=self.current_pose.pose.position.y,
                    z=self.current_pose.pose.position.z
                )
            self.fsm_state = self.StateEnum.APPROACHING
            self.no_detection_counter = 0
        else:
            self.no_detection_counter += 1
            self.get_logger().info(f"[FSM] No targets detected. Counter: {self.no_detection_counter}/{self.max_no_detection_cycles}")
            if self.no_detection_counter >= self.max_no_detection_cycles:
                self.get_logger().info("[FSM] No targets found after %d cycles. Moving to next patch..." % self.max_no_detection_cycles)
                self.current_patch_idx += 1
                self.no_detection_counter = 0
                if self.current_patch_idx >= len(self.patch_positions):
                    self.get_logger().info("[FSM] All patches visited. Returning...")
                    self.fsm_state = self.StateEnum.RETURNING
                else:
                    self.fsm_state = self.StateEnum.MOVING_TO_PATCH

    def sort_targets_by_distance(self, targets, current_position):
        return sorted(targets, key=lambda t: (-t.get('count', 1), math.hypot(
            t['point'].x - current_position.x,
            t['point'].y - current_position.y
        )))

    def recheck_rotation_logic(self):
        now = self.get_clock().now()
        elapsed = (now - self.rotate_start).nanoseconds * 1e-9
        if elapsed < 5.0:
            self.pose.pose.orientation.z = math.sin(math.radians(elapsed * 72) / 2.0)
            self.pose.pose.orientation.w = math.cos(math.radians(elapsed * 72) / 2.0)
            self.setpoint_pub.publish(self.pose)
        else:
            self.get_logger().info("[FSM] Recheck rotation done. Scanning again...")
            self.fsm_state = self.StateEnum.SCANNING
            self.rotate_start = None

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def image_callback(self, msg):
        self.get_logger().info("[CAMERA] Received image")
        self.latest_image_msg = msg

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_image_callback(self, msg):
        self.get_logger().info("[DEPTH] Received depth image")
        self.latest_depth_image_msg = msg
        self.publish_pointcloud()

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_markers(self, targets):
        marker_array = MarkerArray()
        for i, target in enumerate(targets):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "people"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = target['point']
            count = target.get('count', 1)
            marker.scale.x = 0.3 + 0.1 * count
            marker.scale.y = 0.3 + 0.1 * count
            marker.scale.z = 0.3 + 0.1 * count
            marker.color.r = (count > 1) * 1.0
            marker.color.g = (count == 1) * 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_pointcloud(self):
        if self.latest_depth_image_msg is None:
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image_msg, desired_encoding="passthrough")
            rgb_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding="bgr8") if self.latest_image_msg else None
            if rgb_image is not None:
                overlay = rgb_image.copy()
                overlay[::4, ::4, 1] = 255
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_msg.header.stamp = self.get_clock().now().to_msg()
                overlay_msg.header.frame_id = "camera_link"
                self.overlay_pub.publish(overlay_msg)

            height, width = depth_image.shape
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            points = []
            fx = fy = 500.0
            cx = width / 2
            cy = height / 2
            for v in range(0, height, 8):
                for u in range(0, width, 8):
                    z = depth_image[v, u] / 1000.0
                    if 0.1 < z < 10.0:
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append(struct.pack('fff', x, y, z))
            pc2 = PointCloud2()
            pc2.header.frame_id = "camera_link"
            pc2.header.stamp = self.get_clock().now().to_msg()
            pc2.height = 1
            pc2.width = len(points)
            pc2.fields = fields
            pc2.is_bigendian = False
            pc2.point_step = 12
            pc2.row_step = 12 * len(points)
            pc2.is_dense = False
            pc2.data = b''.join(points)
            self.pointcloud_pub.publish(pc2)
        except Exception as e:
            self.get_logger().warn(f"[PC2] Failed to convert depth to PointCloud2: {e}")

    def timer_callback(self):
        self.publish_pose()
        now = self.time_now()
        self.get_logger().info("[FSM] Current position: x=%.2f, y=%.2f, z=%.2f" % (
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z))

        if self.fsm_state == self.StateEnum.WAITING_FOR_FCU:
            if self.current_state.connected:
                self.get_logger().info("[FSM] FCU connected. Moving to ARMING...")
                self.fsm_state = self.StateEnum.ARMING
                self.arm_and_offboard()

        elif self.fsm_state == self.StateEnum.ARMING:
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                self.get_logger().info("[FSM] Armed and in OFFBOARD. Taking off...")
                self.fsm_state = self.StateEnum.TAKING_OFF
                self.takeoff_start = now

        elif self.fsm_state == self.StateEnum.TAKING_OFF:
            if now - self.takeoff_start > Duration(seconds=3.0) and self.current_pose.pose.position.z > 1.8:
                self.get_logger().info("[FSM] Takeoff complete. Altitude: %.2f. Moving to first patch..." % self.current_pose.pose.position.z)
                self.fsm_state = self.StateEnum.MOVING_TO_PATCH
                self.current_patch_idx = 0
            else:
                self.get_logger().info("[FSM] Taking off... Altitude: %.2f" % self.current_pose.pose.position.z)

        elif self.fsm_state == self.StateEnum.MOVING_TO_PATCH:
            if self.current_patch_idx >= len(self.patch_positions):
                self.get_logger().info("[FSM] All patches visited. Returning...")
                self.fsm_state = self.StateEnum.RETURNING
                return
            patch = self.patch_positions[self.current_patch_idx]
            error_x = patch.x - self.current_pose.pose.position.x
            error_y = patch.y - self.current_pose.pose.position.y
            self.pid_error_sum['x'] += error_x
            self.pid_error_sum['y'] += error_y
            control_x = pid_controller(error_x, self.prev_error['x'], self.pid_error_sum['x'], self.pid_gains)
            control_y = pid_controller(error_y, self.prev_error['y'], self.pid_error_sum['y'], self.pid_gains)
            self.prev_error['x'] = error_x
            self.prev_error['y'] = error_y
            self.pose.pose.position.x = patch.x  # Пряме встановлення позиції
            self.pose.pose.position.y = patch.y
            self.pose.pose.position.z = patch.z
            self.setpoint_pub.publish(self.pose)
            dist = math.hypot(error_x, error_y)
            self.get_logger().info("[FSM] Moving to patch #%d: x=%.2f, y=%.2f, dist=%.2f" % (
                self.current_patch_idx, patch.x, patch.y, dist))
            if dist < 0.5:
                self.get_logger().info("[FSM] Reached patch #%d. Rotating to scan..." % self.current_patch_idx)
                self.fsm_state = self.StateEnum.ROTATING
                self.rotate_start = now

        elif self.fsm_state == self.StateEnum.ROTATING:
            now = self.get_clock().now()
            elapsed = (now - self.rotate_start).nanoseconds * 1e-9
            if elapsed < 5.0:
                angle = math.radians(elapsed * 72)
                self.pose.pose.orientation.z = math.sin(angle / 2.0)
                self.pose.pose.orientation.w = math.cos(angle / 2.0)
                self.setpoint_pub.publish(self.pose)
                self.get_logger().info("[FSM] Rotating... Elapsed: %.2f s" % elapsed)
            else:
                self.get_logger().info("[FSM] Rotation done. Proceed to scanning...")
                self.fsm_state = self.StateEnum.SCANNING
                self.scanning_delay_counter = 0
                self.no_detection_counter = 0
                self.rotate_start = None

        elif self.fsm_state == self.StateEnum.SCANNING:
            if self.scanning_delay_counter < self.scanning_delay_max:
                self.get_logger().info("[FSM] Waiting for camera initialization...")
                self.scanning_delay_counter += 1
                return
            self.scanning_state_logic()

        elif self.fsm_state == self.StateEnum.APPROACHING:
            if self.current_target_idx >= len(self.target_positions):
                self.get_logger().info("[FSM] All targets in patch visited. Moving to next patch...")
                self.current_patch_idx += 1
                self.target_positions = []
                self.current_target_idx = 0
                if self.current_patch_idx >= len(self.patch_positions):
                    self.fsm_state = self.StateEnum.RETURNING
                else:
                    self.fsm_state = self.StateEnum.MOVING_TO_PATCH
                return

            target = self.target_positions[self.current_target_idx]
            error_x = target['point'].x - self.current_pose.pose.position.x
            error_y = target['point'].y - self.current_pose.pose.position.y
            self.pid_error_sum['x'] += error_x
            self.pid_error_sum['y'] += error_y
            control_x = pid_controller(error_x, self.prev_error['x'], self.pid_error_sum['x'], self.pid_gains)
            control_y = pid_controller(error_y, self.prev_error['y'], self.pid_error_sum['y'], self.pid_gains)
            self.prev_error['x'] = error_x
            self.prev_error['y'] = error_y
            self.pose.pose.position.x = target['point'].x
            self.pose.pose.position.y = target['point'].y
            self.setpoint_pub.publish(self.pose)
            dist = math.hypot(error_x, error_y)
            if dist < 0.5:
                self.get_logger().info(f"[FSM] Reached target {self.current_target_idx}. Initiating second scan...")
                self.just_visited_position = target['point']
                self.fsm_state = self.StateEnum.POST_APPROACH_ROTATE
                self.rotate_start = now

        elif self.fsm_state == self.StateEnum.POST_APPROACH_ROTATE:
            if (now - self.rotate_start) < Duration(seconds=5.0):
                angle = math.radians(((now - self.rotate_start).nanoseconds * 1e-9) * 72)
                self.pose.pose.orientation.z = math.sin(angle / 2.0)
                self.pose.pose.orientation.w = math.cos(angle / 2.0)
                self.setpoint_pub.publish(self.pose)
            else:
                self.get_logger().info("[FSM] Finished scan after approach. Checking for new targets...")
                self.fsm_state = self.StateEnum.SCANNING
                if self.just_visited_position:
                    self.detected_target_coords.append(self.just_visited_position)
                    self.current_target_idx += 1
                self.just_visited_position = None

        elif self.fsm_state == self.StateEnum.RECHECK_ROTATION:
            self.recheck_rotation_logic()

        elif self.fsm_state == self.StateEnum.RETURNING:
            if self.first_target_position:
                error_x = self.first_target_position.x - self.current_pose.pose.position.x
                error_y = self.first_target_position.y - self.current_pose.pose.position.y
                self.pid_error_sum['x'] += error_x
                self.pid_error_sum['y'] += error_y
                control_x = pid_controller(error_x, self.prev_error['x'], self.pid_error_sum['x'], self.pid_gains)
                control_y = pid_controller(error_y, self.prev_error['y'], self.pid_error_sum['y'], self.pid_gains)
                self.prev_error['x'] = error_x
                self.prev_error['y'] = error_y
                self.pose.pose.position.x = self.first_target_position.x
                self.pose.pose.position.y = self.first_target_position.y
                self.setpoint_pub.publish(self.pose)
                dist = math.hypot(error_x, error_y)
                if dist < 0.5:
                    self.get_logger().info("[FSM] Returned to first position. Landing...")
                    self.fsm_state = self.StateEnum.LANDING
            else:
                self.get_logger().info("[FSM] No first target position. Landing...")
                self.fsm_state = self.StateEnum.LANDING

        elif self.fsm_state == self.StateEnum.LANDING:
            self.pose.pose.position.z = 0.1
            if self.current_pose.pose.position.z < 0.3:
                self.get_logger().info("[FSM] Landing complete. Mission done.")
                self.fsm_state = self.StateEnum.DONE

        elif self.fsm_state == self.StateEnum.DONE:
            self.get_logger().info("[FSM] Mission complete.")
            rclpy.shutdown()

    def arm_and_offboard(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetMode service not available")
            return

        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return

        req_mode = SetMode.Request()
        req_mode.custom_mode = "OFFBOARD"
        self.set_mode_client.call_async(req_mode)
        self.get_logger().info("[FSM] Sent OFFBOARD request")

        req_arm = CommandBool.Request()
        req_arm.value = True
        self.arm_client.call_async(req_arm)
        self.get_logger().info("[FSM] Sent arming request")

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollowerFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[MAIN] Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()