from launch_ros.actions import Node
from launch import LaunchDescription
import os


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'mid360.yaml'
    )
    
    slam_node = Node(
        package='vina_slam',
        executable='vina_slam',
        name='vina_slam',
        output='screen',
        parameters=[config_path],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    rviz_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'rviz_cfg',
        'rviz_cfg.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )

    return LaunchDescription([

        
        slam_node,
        rviz_node,
    ])
