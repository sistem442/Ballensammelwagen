from setuptools import find_packages, setup

package_name = 'yolo_object_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boris',
    maintainer_email='boris@todo.todo',
    description='ROS2 Nodes for YOLO detection, motion control, and ultrasound',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = yolo_object_detection.yolo_detector_node:main',
            'motion_control_node = yolo_object_detection.motion_control_node:main',
            'ultraschall_node = yolo_object_detection.ultraschall_node:main',
            'bale_approach_node = yolo_object_detection.bale_approach:main',
            'shutdown_button_node = yolo_object_detection.shutdown_button:main',
            'message_aggregator = yolo_object_detection.message_aggregator:main'
        ],
    },
)
