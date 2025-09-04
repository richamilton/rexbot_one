import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='rexbot_one'

    map_file = LaunchConfiguration('map_file')

    gazebo_params = os.path.join(
        get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),'config','twist_mux.yaml')

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            # default_value=os.path.join(get_package_share_directory(package_name), 'maps', 'main_floor.yaml'),
            description='Full path to map yaml file'
        ),

        # Start gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', f'params-file:={gazebo_params}', f'verbose:=true'],
            output='screen',
            name='gazebo'
        ),

        # Launch robot state publisher 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Spawn node from the gazebo_ros package.
        # TODO: Add notes for why this node is needed.
        Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                        '-entity', 'my_bot'],
            output='screen'
        ),

        # Launch building with elevator environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('elevator_simulation'),'launch','launch.py'
            )])
        ),

        # NOTE:
        # - controller algos for control manager has to be started separately
        # - controller algos used in this project:
        #       - diff_cont
        #       - joint_broad
        # Start diff drive controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
        ),
        # Start joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
        ),

        # Launch joystick teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )])
        ),

        # Start twist mux
        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        ),

        # NOTE: Enable Navigation
        # 1. Start localization
        # 2. Set initial pose of robot
        # 3. Start navigation
        # Bringup AMCL localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'localization_launch.py'
            )]),
            launch_arguments={
                'map': "ground_floor_map_save.yaml",
                'use_sim_time': 'true',
                'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
            }.items()
        ),
       
        TimerAction(
            period=3.0,  # Wait 3 seconds for AMCL to be ready
            actions=[
                 # Set initial pose
                Node(
                    package='rexbot_one',
                    executable='set_initial_pose_once.py',
                    name='initial_pose_setter',
                    parameters=[
                        {'x': 0.0},
                        {'y': 0.0},
                        {'yaw': 0.0},
                        {'use_sim_time': True}
                    ],
                    output='screen'
                ),
                # Bringup navigation
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name), 'launch', 'navigation_launch.py'
                    )]),
                    launch_arguments={'use_sim_time': 'true', 'subscribe_transient_local': 'true'}.items()
                )
            ]
        ) 
    ])