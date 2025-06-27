from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    planner_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'planner_server.yaml')
    controller_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'controller.yaml')
    bt_navigator_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'bt_navigator.yaml')
    recovery_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'recovery.yaml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
            ),
        Node(
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml]
            ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
            ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'
            ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_path_planner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['planner_server', 
                                        'controller_server', 
                                        'bt_navigator',
                                        'recoveries_server']}])
    ])
