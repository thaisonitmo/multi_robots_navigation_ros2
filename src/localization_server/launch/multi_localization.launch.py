from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import time

def generate_launch_description():
    # time.sleep(1)
    map_file = \
        os.path.join(get_package_share_directory('map_server'), \
        'config', \
        'turtlebot_area_two_robots_clean.yaml')
    tb3_0_config = \
        os.path.join(get_package_share_directory('localization_server'), \
        'config', \
        'tb3_0_amcl_config.yaml')

    tb3_1_config = \
        os.path.join(get_package_share_directory('localization_server'), \
        'config', \
        'tb3_1_amcl_config.yaml')
        
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file} 
                       ]),
        Node(
            namespace='tb3_0',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[tb3_0_config]),
        Node(
            namespace='tb3_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[tb3_1_config]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                            'map_server', 
                            'tb3_0/amcl',
                            'tb3_1/amcl',]}])
    ])
