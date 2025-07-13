from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',  # важливо для правильних топіків
        output='screen',
        parameters=[
            {'fcu_url': 'udp://:14540@localhost:14557'},
            {'config_yaml': '/home/sstarokozhev/ws_offboard_control/src/sptools/config/mavros_config.yaml'},
            {'use_sim_time': True},
            {'enable_timesync': False}
        ]
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=[
            '/world/new_custom_world_v3/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_setpoint_local = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_setpoint_local',
        arguments=[
            '/mavros/setpoint_position/local'
            '@geometry_msgs/msg/PoseStamped@gz.msgs.Pose',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        #mavros_node,
        bridge_imu,
        bridge_setpoint_local
    ])
