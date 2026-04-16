import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    default_rviz_config = os.path.join(
        get_package_share_directory('indoor_drone_sim'),
        'rviz',
        'indoor_drone_sim.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config),
        Node(
            package='indoor_drone_sim',
            executable='sim_world_node',
            name='sim_world_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='indoor_drone_sim',
            executable='sim_server_node',
            name='sim_server_node',
            output='screen',
            emulate_tty=True,
        ),
        # Node(
        #     package='indoor_drone_sim',
        #     executable='radmapper_node',
        #     name='radmapper_node',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
        ),
    ])
