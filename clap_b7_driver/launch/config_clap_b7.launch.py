import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('clap_b7_driver'),
        'config',
        'config_clap_b7.param.yaml'
    )

    return LaunchDescription([
        Node(
            package='clap_b7_driver',
            name="clap_b7_config",
            executable = 'config_clap_b7_node',
            output = 'screen',
            parameters=[config]
        )
    ])