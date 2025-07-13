import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import math
from ultralytics import YOLO

def detect_humans(image_msg, depth_msg, bridge, current_position, yaw_angle, image_center=(960, 540), logger=None, model=None):
    targets = []
    try:
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        depth_frame = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough') if depth_msg else None
        if model is None:
            model = YOLO('yolov8n.pt')
        results = model(frame, imgsz=960, conf=0.2)  # Збільшено роздільну здатність, знижено поріг
        boxes = results[0].boxes.xywh[results[0].boxes.cls == 0]

        image_width, image_height = frame.shape[1], frame.shape[0]
        if depth_frame is not None:
            depth_width, depth_height = depth_frame.shape[1], depth_frame.shape[0]
            scale_x = depth_width / image_width
            scale_y = depth_height / image_height
        else:
            scale_x = scale_y = 1.0

        if logger:
            logger.info(f"[YOLO] Found {len(boxes)} boxes")

        for i, box in enumerate(boxes):
            cx, cy, w, h = box
            cx, cy = int(cx), int(cy)
            if logger:
                logger.info(f"[DETECT] Людина #{i} в пікселях: ({cx}, {cy})")

            depth_cx = int(cx * scale_x)
            depth_cy = int(cy * scale_y)
            distance_to_target = 2.0
            if depth_frame is not None and 0 <= depth_cy < depth_height and 0 <= depth_cx < depth_width:
                distance_to_target = depth_frame[depth_cy, depth_cx] / 1000.0
                if not (0.1 < distance_to_target < 15.0):
                    distance_to_target = 2.0

            dx_pixel = cx - image_center[0]
            fov_horizontal = math.radians(60)
            pixel_to_radian = fov_horizontal / image_width
            angle_x = dx_pixel * pixel_to_radian
            target_x = current_position.x + distance_to_target * math.cos(yaw_angle + angle_x)
            target_y = current_position.y + distance_to_target * math.sin(yaw_angle + angle_x)
            target_z = current_position.z

            target = {
                'point': Point(x=target_x, y=target_y, z=target_z),
                'count': 1
            }
            targets.append(target)
            if logger:
                logger.info(f"[DETECT] Ціль #{i} додана: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}, dist={distance_to_target:.2f}")

        if logger:
            logger.info(f"[YOLO] {len(targets)} detections accepted")
            cv2.imwrite(f"/tmp/yolo_input_{image_msg.header.stamp.sec}_{image_msg.header.stamp.nanosec}.jpg", frame)

    except Exception as e:
        if logger:
            logger.error(f"[DETECT] Помилка обробки зображення: {e}")

    return targets

def simulate_target_detection(current_position, altitude, logger=None):
    """
    Симулює виявлення цілі на відстані 2 метри від дрона.
    Повертає список із однією ціллю.

    Args:
        current_position: Поточна позиція дрона (geometry_msgs/Point).
        altitude: Висота польоту (float).
        logger: Логер для дебаг-повідомлень (rclpy.logger).

    Returns:
        list: Список із однією ціллю {'id': None, 'point': Point}.
    """
    if logger:
        logger.info("[SEARCH] Симуляція: Ціль знайдена")

    target = {
        'id': None,  # ID буде присвоєно в HumanFollower
        'point': Point(
            x=current_position.x + 2.0,
            y=current_position.y,
            z=altitude
        )
    }
    if logger:
        logger.info(f"[SEARCH] Симульована ціль: x={target['point'].x:.2f}, y={target['point'].y:.2f}")
    return [target]