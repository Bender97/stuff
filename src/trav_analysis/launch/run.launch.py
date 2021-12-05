import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser

def generate_launch_description():

    # Configure environment
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    logging_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}')


    # Simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes Configurations
    config_file = os.path.join(get_package_share_directory('trav_analysis'), 'config', 'params.yaml')

    predicter_node = Node(
        package="trav_analysis",
        executable="trav_analysis_predicter",
        output="screen",
        parameters=[config_file],
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    )


    config_file = os.path.join(get_package_share_directory('lego_loam_sr'), 'config', 'loam_config.yaml')

    # Tf transformations
    transform_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_map',
        arguments=['0', '0', '0', '1.570795', '0', '1.570795', 'map', 'camera_init'],
    )

    transform_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', 'camera', 'base_link'],
    )

    transform_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # LeGO-LOAM
    lego_loam_node = Node(
        package='lego_loam_sr',
        executable='lego_loam_sr',
        output='screen',
        parameters=[config_file],
        remappings=[('/lidar_points', '/velodyne_points')],
    )


    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(logging_envvar)

    # Add nodes
    # ld.add_action(lego_loam_node)
    # ld.add_action(transform_base_link)
    # ld.add_action(transform_map)
    # ld.add_action(transform_camera)
    ld.add_action(predicter_node)

    return ld