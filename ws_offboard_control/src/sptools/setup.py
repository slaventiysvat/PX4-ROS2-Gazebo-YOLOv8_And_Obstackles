from setuptools import find_packages, setup
import sys


package_name = 'sptools'

package_name = 'sptools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    python_requires='>=3.10, <3.11',  # Обмежує до Python 3.10
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mavros_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/human_follower.launch.py', 
                                                'launch/minimal_takeoff.launch.py', 
                                                'launch/minimal_mavros_gz.launch.py',
                                                'launch/px4_gz_sim.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'mavros_msgs', 'sensor_msgs', 'opencv-python', 'numpy', 'ultralytics', 'scikit-learn'],
    zip_safe=True,
    maintainer='sstarokozhev',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for drone control and human following',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_follower = sptools.human_follower:main',
            'arming_node = sptools.arming_node:main',
            'set_home = sptools.set_home:main',
            'minimal_takeoff = sptools.minimal_takeoff:main',
            'test_timer = sptools.test_timer:main',
        ],
    },
    package_data={
        package_name: ['sptools/*.py', 'resource/*', 'config/*', 'launch/*'],
    },
)
