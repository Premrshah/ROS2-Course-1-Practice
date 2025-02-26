from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mars_rover_tasks',
            executable='subscriber_obstacle_detector_executable',
            output='screen'),
        Node(
            package='mars_rover_tasks',
            executable='publish_mars_rover_move_executable',
            ouptut='screen')
    ])