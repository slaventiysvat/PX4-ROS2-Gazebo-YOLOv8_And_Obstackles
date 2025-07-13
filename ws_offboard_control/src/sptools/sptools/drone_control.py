import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from sptools.utils import compute_yaw_quaternion, compute_distance, pid_controller

def publish_setpoint(publisher, x, y, z, yaw=0.0, node=None):
    """
    Публікує setpoint для дрона.

    Args:
        publisher: ROS-паблішер для /mavros/setpoint_position/local (rclpy.publisher.Publisher).
        x, y, z: Координати setpoint (float).
        yaw: Кут yaw у радіанах (float, за замовчуванням 0.0).
        node: Вузол ROS для доступу до часу (rclpy.node.Node, опціонально).
    """
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg() if node else PoseStamped().header.stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quat = compute_yaw_quaternion(yaw)
    pose.pose.orientation.x = quat['x']
    pose.pose.orientation.y = quat['y']
    pose.pose.orientation.z = quat['z']
    pose.pose.orientation.w = quat['w']
    publisher.publish(pose)

def handle_takeoff(node, state, setpoint_pub, set_mode_client, arming_client, params):
    publish_setpoint(setpoint_pub, node.home_position.x, node.home_position.y, params['altitude'], node=node)
    node.setpoint_counter += 1

    if node.setpoint_counter == 1:
        node.get_logger().info("[TAKEOFF] Початок зльоту")
    elif node.setpoint_counter % 50 == 0:
        node.get_logger().info(f"[TAKEOFF] Setpoint {node.setpoint_counter}, висота={node.current_altitude:.2f}")

    # Надіслати OFFBOARD, якщо ще не встановлено
    if not node.offboard_set and node.setpoint_counter >= 20:
        if set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = set_mode_client.call_async(req)
            future.add_done_callback(node.set_mode_callback)
            node.get_logger().info("[TAKEOFF] Надсилаємо запит на OFFBOARD")
        else:
            node.get_logger().warn("[TAKEOFF] Сервіс set_mode недоступний")

    # Озброєння
    if state.mode == "OFFBOARD" and not state.armed:
        if node.arming_retries >= params['max_arming_retries']:
            node.get_logger().error("[TAKEOFF] Перевищено спроб озброєння → LAND")
            return "LAND"
        if arming_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            future = arming_client.call_async(req)
            future.add_done_callback(node.arming_callback)
            node.arming_retries += 1
            node.get_logger().info(f"[TAKEOFF] Спроба ARM #{node.arming_retries}")
        else:
            node.get_logger().warn("[TAKEOFF] Сервіс ARM недоступний")

    # Перехід у SEARCH
    if state.armed and node.current_altitude >= params['takeoff_confirm_alt']:
        node.takeoff_confirmed = True
        node.get_logger().info("[TAKEOFF] Зліт підтверджено → SEARCH")
        node.timer_start = node.get_clock().now()
        node.rotation_start_yaw = node.yaw_angle
        node.full_rotation = False
        node.search_moves = 0
        node.grid_index = 0
        return "SEARCH"

    return "TAKEOFF"



def handle_search(node, setpoint_pub, params):
    """
    Керує станом SEARCH: виконує повний оберт для пошуку цілей.

    Args:
        node: Вузол ROS (HumanFollower).
        setpoint_pub: Паблішер для setpoint (rclpy.publisher.Publisher).
        params: Словник із параметрами (altitude, yaw_rate, search_duration, max_search_moves).

    Returns:
        str: Новий стан ("SEARCH", "FOLLOW", "SEARCH_GRID", "RETURN").
    """
    node.yaw_angle += params['yaw_rate'] * 0.2
    node.yaw_angle = node.yaw_angle % (2 * math.pi)
    publish_setpoint(setpoint_pub, node.current_position.x, node.current_position.y, params['altitude'], node.yaw_angle, node=node)

    yaw_diff = abs((node.yaw_angle - node.rotation_start_yaw) % (2 * math.pi))
    node.get_logger().debug(f"[SEARCH] yaw_diff={yaw_diff:.3f}, targets={len(node.targets)}, search_moves={node.search_moves}")

    if node.targets:
        node.get_logger().info(f"[SEARCH] Знайдено {len(node.targets)} цілей, перехід до FOLLOW")
        node.search_moves = 0
        node.grid_index = 0
        node.follow_index = 0
        node.hover_start_time = node.get_clock().now()
        node.full_rotation = False
        node.timer_start = node.get_clock().now()
        return "FOLLOW"

    if yaw_diff >= 2 * math.pi - 0.01:
        node.full_rotation = True
        node.get_logger().info(f"[SEARCH] Завершено повний оберт, цілі не знайдено, перехід до SEARCH_GRID")
        node.search_moves += 1
        node.full_rotation = False
        node.target_found = False
        node.timer_start = node.get_clock().now()
        if node.grid_index < len(node.grid_points):
            node.search_target.x = node.grid_points[node.grid_index][0]
            node.search_target.y = node.grid_points[node.grid_index][1]
            node.search_target.z = params['altitude']
            node.grid_index += 1
            return "SEARCH_GRID"
        else:
            node.get_logger().info("[SEARCH] Досягнуто кінець сітки, перехід до RETURN")
            node.search_moves = 0
            node.grid_index = 0
            return "RETURN"

    if node.log_counter % 10 == 0:
        node.get_logger().info(f"[SEARCH] Сканування, кут yaw={math.degrees(node.yaw_angle):.1f}°, переміщення={node.search_moves}/{params['max_search_moves']}")

    elapsed = (node.get_clock().now() - node.timer_start).nanoseconds * 1e-9
    if elapsed > params['search_duration']:
        node.get_logger().info(f"[SEARCH] Ціль не знайдено за {params['search_duration']} сек, перехід до RETURN")
        node.search_moves = 0
        node.grid_index = 0
        return "RETURN"

    return "SEARCH"

def handle_search_grid(node, setpoint_pub, params):
    """
    Керує станом SEARCH_GRID: переміщає дрон до наступної точки сітки.

    Args:
        node: Вузол ROS (HumanFollower).
        setpoint_pub: Паблішер для setpoint (rclpy.publisher.Publisher).
        params: Словник із параметрами (altitude, search_duration, max_search_moves).

    Returns:
        str: Новий стан ("SEARCH", "RETURN").
    """
    publish_setpoint(setpoint_pub, node.search_target.x, node.search_target.y, params['altitude'], node.yaw_angle, node=node)

    dist = compute_distance(node.current_position.x, node.current_position.y, node.search_target.x, node.search_target.y)
    if node.log_counter % 10 == 0:
        node.get_logger().info(f"[SEARCH_GRID] Політ до точки: x={node.search_target.x:.2f}, y={node.search_target.y:.2f}, d={dist:.2f}, точка={node.grid_index}/{len(node.grid_points)}")

    if dist < 0.5:
        node.get_logger().info(f"[SEARCH_GRID] Досягнуто точку {node.grid_index}/{len(node.grid_points)}, перехід до SEARCH")
        node.rotation_start_yaw = node.yaw_angle
        node.full_rotation = False
        node.timer_start = node.get_clock().now()
        return "SEARCH"

    elapsed = (node.get_clock().now() - node.timer_start).nanoseconds * 1e-9
    if elapsed > params['search_duration'] or node.search_moves >= params['max_search_moves']:
        node.get_logger().info(f"[SEARCH_GRID] Ціль не знайдено за {params['search_duration']} сек або досягнуто максимум переміщень, перехід до RETURN")
        node.search_moves = 0
        node.grid_index = 0
        return "RETURN"

    return "SEARCH_GRID"

def handle_follow(node, setpoint_pub, params):
    """
    Керує станом FOLLOW: політ до цілей із PID-регулятором.

    Args:
        node: Вузол ROS (HumanFollower).
        setpoint_pub: Паблішер для setpoint (rclpy.publisher.Publisher).
        params: Словник із параметрами (altitude, hover_duration).

    Returns:
        str: Новий стан ("FOLLOW", "SEARCH").
    """
    if not node.targets or node.follow_index >= len(node.targets):
        node.get_logger().info("[FOLLOW] Немає цілей або індекс поза межами, перехід до SEARCH")
        return "SEARCH"

    target = node.targets[node.follow_index]
    target_pos = target['point']
    current_pos = node.current_position

    # PID обчислення
    error_x = target_pos.x - current_pos.x
    error_y = target_pos.y - current_pos.y
    node.pid_error_sum['x'] += error_x
    node.pid_error_sum['y'] += error_y
    control_x = (node.pid_gains['kp'] * error_x + 
                 node.pid_gains['ki'] * node.pid_error_sum['x'] + 
                 node.pid_gains['kd'] * (error_x - node.prev_error['x']))
    control_y = (node.pid_gains['kp'] * error_y + 
                 node.pid_gains['ki'] * node.pid_error_sum['y'] + 
                 node.pid_gains['kd'] * (error_y - node.prev_error['y']))
    
    control_x = max(min(control_x, 1.0), -1.0)  # Обмеження до [-1, 1] м/с
    control_y = max(min(control_y, 1.0), -1.0)

    node.prev_error['x'] = error_x
    node.prev_error['y'] = error_y

    # Надсилання setpoint
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = current_pos.x + control_x
    pose.pose.position.y = current_pos.y + control_y
    pose.pose.position.z = params['altitude']
    pose.pose.orientation.w = 1.0
    setpoint_pub.publish(pose)

    # Перевірка близькості до цілі
    dist = math.sqrt(error_x**2 + error_y**2)
    if dist < 0.5:
        node.get_logger().info(f"[FOLLOW] Досягнуто ціль #{target['id']}, перехід до SEARCH")
        node.visited_targets.append(target)
        node.follow_index += 1
        return "SEARCH"

    return "FOLLOW"

def handle_return(node, setpoint_pub, params):
    """
    Керує станом RETURN: повернення до початкової позиції.

    Args:
        node: Вузол ROS (HumanFollower).
        setpoint_pub: Паблішер для setpoint (rclpy.publisher.Publisher).
        params: Словник із параметрами (altitude).

    Returns:
        str: Новий стан ("RETURN", "LAND").
    """
    publish_setpoint(setpoint_pub, node.home_position.x, node.home_position.y, params['altitude'], node=node)

    dist = compute_distance(node.current_position.x, node.current_position.y, node.home_position.x, node.home_position.y)
    if node.log_counter % 10 == 0:
        node.get_logger().info(f"[RETURN] До початкової: x={node.home_position.x:.2f}, y={node.home_position.y:.2f}, d={dist:.2f}")

    if dist < 0.5:
        node.get_logger().info("[RETURN] Повернення завершено → LAND")
        return "LAND"

    return "RETURN"

def handle_land(node, setpoint_pub, arming_client, params):
    """
    Керує станом LAND: посадка та роззброєння.

    Args:
        node: Вузол ROS (HumanFollower).
        setpoint_pub: Паблішер для setpoint (rclpy.publisher.Publisher).
        arming_client: Клієнт для армування (rclpy.client.Client).
        params: Словник із параметрами (none).

    Returns:
        str: Новий стан ("LAND", "IDLE").
    """
    publish_setpoint(setpoint_pub, node.current_position.x, node.current_position.y, 0.1, node=node)

    if node.current_altitude < 0.15:
        if arming_client.wait_for_service(timeout_sec=2.0):
            req = CommandBool.Request()
            req.value = False
            node.get_logger().info("[LAND] Виклик роззброєння")
            arming_client.call_async(req)
            node.get_logger().info("[LAND] Посадка завершена, роззброєння")
        node.idle_start_time = node.get_clock().now()
        return "IDLE"
    else:
        if node.log_counter % 10 == 0:
            node.get_logger().info(f"[LAND] Посадка, висота={node.current_altitude:.2f}")

    return "LAND"

def handle_idle(node, setpoint_pub, params):
    """
    Керує станом IDLE: очікування перед повторним зльотом.

    Args:
        node: Вузол ROS (HumanFollower).
        setpoint_pub: Паблішер для setpoint (rclpy.publisher.Publisher).
        params: Словник із параметрами (idle_timeout).

    Returns:
        str: Новий стан ("IDLE", "TAKEOFF").
    """
    if not node.idle_logged:
        node.get_logger().info("[IDLE] Дрон в стані IDLE")
        node.idle_logged = True
    publish_setpoint(setpoint_pub, node.current_position.x, node.current_position.y, 0.1, node=node)

    elapsed = (node.get_clock().now() - node.idle_start_time).nanoseconds * 1e-9
    if elapsed > params['idle_timeout'] and node.state.connected and node.state.system_status >= 3:
        node.get_logger().info("[IDLE] Таймер очікування завершився, перехід до TAKEOFF")
        node.setpoint_counter = 0
        node.arming_retries = 0
        node.offboard_set = False
        node.takeoff_confirmed = False
        node.targets = []
        node.visited_targets = []
        node.target_id_counter = 0
        node.full_rotation = False
        node.search_moves = 0
        node.grid_index = 0
        node.idle_logged = False
        node.timer_start = node.get_clock().now()
        return "TAKEOFF"

    return "IDLE"