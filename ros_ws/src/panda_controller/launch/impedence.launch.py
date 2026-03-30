from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panda_mujoco_bridge',
            executable='mujoco_bridge',
            name='mujoco_bridge',
            output='screen',
        ),
        Node(
            package='panda_controller',
            executable='impedence',
            name='impedence_controller',
            output='screen',
        ),
    ])