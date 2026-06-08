from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def _optional_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be a boolean value, got: {value}')


def _launch_setup(context, *args, **kwargs):
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

    debug_overrides = {}
    debug_enable_z_drift_log = LaunchConfiguration(
        'debug_enable_z_drift_log'
    ).perform(context)
    if debug_enable_z_drift_log != '':
        debug_overrides['Debug.enable_z_drift_log'] = _optional_bool(
            debug_enable_z_drift_log,
            'debug_enable_z_drift_log'
        )

    debug_run_label = LaunchConfiguration('debug_run_label').perform(context)
    if debug_run_label != '':
        debug_overrides['Debug.run_label'] = debug_run_label

    debug_log_root = LaunchConfiguration('debug_log_root').perform(context)
    if debug_log_root != '':
        debug_overrides['Debug.log_root'] = debug_log_root

    debug_fail_on_frontend_degenerate = LaunchConfiguration(
        'debug_fail_on_frontend_degenerate'
    ).perform(context)
    if debug_fail_on_frontend_degenerate != '':
        debug_overrides['Debug.fail_on_frontend_degenerate'] = _optional_bool(
            debug_fail_on_frontend_degenerate,
            'debug_fail_on_frontend_degenerate'
        )

    node_parameters = [vina_config_path]
    if debug_overrides:
        node_parameters.append(debug_overrides)

    # VINA-SLAM node
    vina_slam_node = Node(
        package='vina_slam',
        executable='vina_slam',
        name='vina_slam',
        output='screen',
        parameters=node_parameters,
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

    return [vina_slam_node, rviz_node]


def generate_launch_description() -> LaunchDescription:
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'vina_config',
        default_value='mid360.yaml',
        # default_value='velodyne.yaml',
        # default_value='robosense_airy.yaml',
        description='VINA-SLAM config file name'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    debug_enable_z_drift_log_arg = DeclareLaunchArgument(
        'debug_enable_z_drift_log',
        default_value='',
        description='Optional override for Debug.enable_z_drift_log'
    )

    debug_run_label_arg = DeclareLaunchArgument(
        'debug_run_label',
        default_value='',
        description='Optional override for Debug.run_label'
    )

    debug_log_root_arg = DeclareLaunchArgument(
        'debug_log_root',
        default_value='',
        description='Optional override for Debug.log_root'
    )

    debug_fail_on_frontend_degenerate_arg = DeclareLaunchArgument(
        'debug_fail_on_frontend_degenerate',
        default_value='',
        description='Optional override for Debug.fail_on_frontend_degenerate'
    )

    return LaunchDescription([
        config_arg,
        launch_rviz_arg,
        debug_enable_z_drift_log_arg,
        debug_run_label_arg,
        debug_log_root_arg,
        debug_fail_on_frontend_degenerate_arg,
        OpaqueFunction(function=_launch_setup),
    ])
