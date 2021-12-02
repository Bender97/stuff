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




    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)

    # Add nodes
    ld.add_action(predicter_node)

    return ld