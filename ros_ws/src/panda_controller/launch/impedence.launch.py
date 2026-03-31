from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    axes_on_arg = DeclareLaunchArgument(
        'axes_on',
        default_value='false',
        description='Enable axes visualization'
    )
    
    return LaunchDescription([
        axes_on_arg,
        Node(
            package='panda_controller',
            executable='impedence',
            name='impedence_controller',
            output='screen',
        ),
        Node(
            package='panda_mujoco_bridge',
            executable='mujoco_bridge',
            name='mujoco_bridge',
            output='screen',
            parameters=[{'axes_on': LaunchConfiguration('axes_on')}],
        ),
    ])