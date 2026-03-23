from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'vina_config',
        # default_value='mid360.yaml',
        default_value='velodyne.yaml',
        description='VINA-SLAM config file name'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # VINA-SLAM config path
    vina_config_path = PathJoinSubstitution([
        FindPackageShare('vina_slam'),
        'config',
        LaunchConfiguration('vina_config')
    ])

    # RViz config path
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('vina_slam'),
        'rviz_cfg',
        'rviz_cfg.rviz'
    ])

    # VINA-SLAM node
    vina_slam_node = Node(
        package='vina_slam',
        executable='vina_slam',
        name='vina_slam',
        output='screen',
        parameters=[vina_config_path],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        config_arg,
        launch_rviz_arg,
        vina_slam_node,
        rviz_node,
    ])
