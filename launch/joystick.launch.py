from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('rexbot_one'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[joy_params],
            # NOTE:
            # - ros2 control listens for velocity commands on the diff_cont/cmd_vel_unstamped topic
            # - remap the cmd_vel topic to the diff_cont/cmd_vel_unstamped topic
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])