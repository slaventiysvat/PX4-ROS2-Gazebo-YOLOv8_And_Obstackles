from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            # Основна камера
            '/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Глибинна камера
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            # Глибинна камера (точкова хмара)
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/image', '/camera'),
            ('/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', '/camera_info'),
            ('/depth_camera', '/depth/image_raw'),
            ('/depth_camera/points', '/depth/pointcloud')
        ],
        parameters=[{'use_sim_time': True}]
    )

    human_follower_node = Node(
        package='sptools',
        executable='human_follower',
        output='screen',
       #parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_bridge,
        human_follower_node,
    ])

#x500_depth_0/camera_link/StereoOV7251

#ros2 run ros_gz_bridge parameter_bridge 
#/camera@sensor_msgs/msg/Image@ignition.msgs.Image 
#/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo
#/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked 
#/depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image
