from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{'image_size': [640, 480]}],
            remappings=[('image_raw', '/image_raw')]
        ),

        Node(
            package='yolo_object_detection',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen'
        ),

        Node(
            package='yolo_object_detection',
            executable='motion_control_node',
            name='motion_control',
            output='screen'
        ),

        Node(
            package='yolo_object_detection',
            executable='bale_approach_node',
            name='bale_approach',
            output='screen'
        ),

        Node(
            package='yolo_object_detection',
            executable='shutdown_button_node',
            name='system_shutdown',
            output='screen'
        ),

        Node(
            package='yolo_object_detection',
            executable='message_aggregator',
            name='message_aggregator',
            output='screen'
        ),

        Node(
           package='yolo_object_detection',
           executable='ultraschall_node',
           name='ultraschall_node',
           output='screen'
       ),
    ])
