import math

def compute_yaw_quaternion(yaw):
    """
    Обчислює кватерніон для заданого кута yaw.

    Args:
        yaw: Кут yaw у радіанах (float).

    Returns:
        dict: Словник із кватерніоном {'x': float, 'y': float, 'z': float, 'w': float}.
    """
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }

def compute_distance(x1, y1, x2, y2):
    """
    Обчислює евклідову відстань між двома точками.

    Args:
        x1, y1: Координати першої точки (float).
        x2, y2: Координати другої точки (float).

    Returns:
        float: Відстань між точками.
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def pid_controller(error, prev_error, error_sum, gains, dt=0.1):
    """
    Реалізує PID-регулятор для керування.

    Args:
        error: Поточна помилка (float).
        prev_error: Попередня помилка (float).
        error_sum: Сума помилок для інтегральної складової (float).
        gains: Словник із коефіцієнтами {'kp': float, 'ki': float, 'kd': float}.
        dt: Часовий крок, за замовчуванням 0.1 с (float).

    Returns:
        float: Вихідний сигнал PID у межах [-1.0, 1.0].
    """
    derivative = (error - prev_error) / dt
    output = gains['kp'] * error + gains['ki'] * error_sum * dt + gains['kd'] * derivative
    return max(min(output, 1.0), -1.0)

def check_target_duplicate(target_x, target_y, existing_targets):
    """
    Перевіряє, чи нова ціль уже існує серед наявних (відстань < 0.5 м).

    Args:
        target_x, target_y: Координати нової цілі (float).
        existing_targets: Список словників [{'id': int, 'point': Point}, ...].

    Returns:
        bool: True, якщо ціль нова; False, якщо вже існує.
    """
    for target in existing_targets:
        dist = compute_distance(target['point'].x, target['point'].y, target_x, target_y)
        if dist < 0.5:
            return False
    return True