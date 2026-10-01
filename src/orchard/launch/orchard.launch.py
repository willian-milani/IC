import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # CAMINHOS
    # ============================================================

    orchard_share = get_package_share_directory('orchard')

    orchard_worlds = os.path.join(
        orchard_share,
        'worlds'
    )

    orchard_models = os.path.join(
        orchard_share,
        'models'
    )

    clearpath_gz_share = get_package_share_directory(
        'clearpath_gz'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    # Mundo personalizado
    orchard_world = os.path.join(
        orchard_worlds,
        'orchard.sdf'
    )

    # ============================================================
    # ARGUMENTOS
    # ============================================================

    rviz = LaunchConfiguration('rviz')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # ============================================================
    # CAMINHO DE RECURSOS DO GAZEBO
    # ============================================================

    # Mantém os recursos do Clearpath e adiciona:
    # - modelos do nosso pomar
    # - mundo do nosso pomar

    current_gz_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    resource_paths = [
        orchard_models,
        orchard_worlds,
        os.path.join(clearpath_gz_share, 'worlds'),
        os.path.join(clearpath_gz_share, 'meshes'),
    ]

    if current_gz_resource_path:
        resource_paths.append(current_gz_resource_path)

    gz_resource_path = ':'.join(resource_paths)

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # ============================================================
    # GAZEBO SIM
    # ============================================================

    gz_sim_launch = os.path.join(
        ros_gz_sim_share,
        'launch',
        'gz_sim.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gz_sim_launch
        ),
        launch_arguments={
            'gz_args': orchard_world + ' -r -v 4'
        }.items()
    )

    # ============================================================
    # CLEARPATH ROBOT SPAWN
    # ============================================================

    orchard_husky_share = get_package_share_directory(
        'orchard_husky'
    )

    robot_spawn_launch = os.path.join(
        orchard_husky_share,
        'launch',
        'spawn_husky.launch.py'
    )

    robot_spawn = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_spawn_launch
                ),

                launch_arguments={
                    'x': x,
                    'y': y,
                    'z': z,
                    'yaw': yaw,
                }.items()
            )
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/sensors/seyond_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/sensors/seyond_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
)

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
)

    # ============================================================
    # DESCRIÇÃO DO LAUNCH
    # ============================================================

    return LaunchDescription([

        # Argumentos
        DeclareLaunchArgument(
            'rviz',
            default_value='false'
        ),

        DeclareLaunchArgument('x', default_value='-0.0'),
        DeclareLaunchArgument('y', default_value='-12.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='1.57'),


        # Recursos do Gazebo
        set_gz_resource_path,

        # Inicia o nosso mundo
        gazebo,

        clock_bridge,
        
        lidar_bridge,

        imu_bridge,

        # Insere o robô Clearpath
        robot_spawn,
    ])