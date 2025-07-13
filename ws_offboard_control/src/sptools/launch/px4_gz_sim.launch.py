from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Шляхи
    px4_dir = os.path.expanduser("~/PX4-Autopilot")  # Змініть на ваш шлях до PX4
    world_path = os.path.join(px4_dir, "Tools/simulation/gz/worlds/new_custom_world_v3.sdf")  # Шлях до вашого світу
    model_pose = "268.08,-128.22,3.86,0,0,-0.7"  # Позиція дрона

    # Аргументи
    declare_world = DeclareLaunchArgument(
        "world", default_value=world_path, description="Path to Gazebo world file"
    )

    # Запуск Gazebo з світом
    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-s", LaunchConfiguration("world")
        ],
        output="screen",
    )

    # Спавн моделі дрона
    spawn_model = ExecuteProcess(
        cmd=[
            "gz", "model", "--spawn",
            "--model-name", "x500_depth",
            "--pose", model_pose,
            "--file", os.path.join(px4_dir, "Tools/simulation/gz/models/x500_depth/model.sdf")
        ],
        output="screen",
    )

    # Запуск PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=[
            FindExecutable(name="bash"), "-c",
            f"PX4_GZ_LOG_LEVEL=debug PX4_LOCKSTEP=1 PX4_SYS_AUTOSTART=4002 "
            f"PX4_SIM_MODEL=gz_x500_depth PX4_GZ_MODEL_POSE='{model_pose}' "
            f"PX4_SIMULATOR=GZ PX4_GZ_WORLD=new_custom_world_v3 "
            f"{px4_dir}/build/px4_sitl_default/bin/px4"
        ],
        output="screen",
        cwd=px4_dir,
    )

    # Запуск MAVROS
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            {"fcu_url": "udp://:14540@localhost:14557"},
            {"config_yaml": os.path.join(
                get_package_share_directory("sptools"), "config", "mavros_config.yaml"
            )},
            {"state_frequency": 30.0},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        declare_world,
        gazebo,
        spawn_model,
        px4_sitl,
        mavros,
    ])