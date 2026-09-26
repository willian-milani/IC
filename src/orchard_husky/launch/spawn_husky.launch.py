import os
import xacro



from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.actions import ExecuteProcess

from launch.actions import TimerAction

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    pkg = get_package_share_directory(
        'orchard_husky'
    )


    xacro_file = os.path.join(
        pkg,
        'urdf',
        'husky_minimal.xacro'
    )


    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'is_sim': 'true'
        }
    ).toxml()


    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'orchard_husky',
            '-topic',
            'robot_description',
            '-x',
            x,
            '-y',
            y,
            '-z',
            z,
            '-Y',
            yaw
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    platform_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'platform_velocity_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'x',
            default_value='-20.0'
        ),

        DeclareLaunchArgument(
            'y',
            default_value='-12.0'
        ),

        DeclareLaunchArgument(
            'z',
            default_value='0.3'
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0'
        ),


        robot_state_publisher,

        spawn_robot,

        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                platform_controller_spawner]
        )

    ])