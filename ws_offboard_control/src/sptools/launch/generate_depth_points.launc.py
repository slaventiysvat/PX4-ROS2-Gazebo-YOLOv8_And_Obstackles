from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge: Depth Image (remapped to ROS-friendly name)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_depth_image',
            arguments=[
                '/world/new_custom_world_v3/model/x500_depth_0/link/camera_link/sensor/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '--ros-args', '--remap',
                '/world/new_custom_world_v3/model/x500_depth_0/link/camera_link/sensor/depth_camera/image:=/camera/depth/image_raw'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Bridge: Camera Info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_camera_info',
            arguments=[
                '/world/new_custom_world_v3/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '--ros-args', '--remap',
                '/world/new_custom_world_v3/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/camera/depth/camera_info'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # ROS 2 Node: Convert depth image + camera info → point cloud
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz',
            name='depth_to_pointcloud',
            remappings=[
                ('image_rect', '/camera/depth/image_raw'),
                ('camera_info', '/camera/depth/camera_info'),
                ('points', '/depth_points')
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
