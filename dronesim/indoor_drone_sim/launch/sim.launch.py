from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='indoor_drone_sim',
            executable='sim_world_node',
            name='sim_world_node',
            output='screen',
            parameters=[{
                'run_without_rviz': True,
                'camera_backend': 'gpu',
            }]
        ),
        Node(
            package='indoor_drone_sim',
            executable='sim_server_node',
            name='sim_server_node',
            output='screen',
        ),
        # Node(
        #     package='indoor_drone_sim',
        #     executable='radmapper_node',
        #     name='radmapper_node',
        #     output='screen',
        # ),
        Node(
            package='indoor_drone_sim',
            executable='toucher_node',
            name='toucher_node',
            output='screen',
        ),
    ])
