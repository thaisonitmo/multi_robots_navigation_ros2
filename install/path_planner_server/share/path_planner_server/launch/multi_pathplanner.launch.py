from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
# import time

def generate_launch_description():
    tb3_0_planner_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_0_planner_server.yaml')
    tb3_0_controller_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_0_controller.yaml')
    tb3_0_bt_navigator_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_0_bt_navigator.yaml')
    tb3_0_recovery_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_0_recovery.yaml')

    tb3_1_planner_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_1_planner_server.yaml')
    tb3_1_controller_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_1_controller.yaml')
    tb3_1_bt_navigator_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_1_bt_navigator.yaml')
    tb3_1_recovery_yaml = \
        os.path.join(get_package_share_directory('path_planner_server'), \
        'config', \
        'tb3_1_recovery.yaml')

    return LaunchDescription([
        #tb3_0
        Node(
            namespace='tb3_0',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[tb3_0_planner_yaml]
            ),
        Node(
            namespace='tb3_0',
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[tb3_0_controller_yaml]
            ),
        Node(
            namespace='tb3_0',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[tb3_0_bt_navigator_yaml]
            ),
        Node(
            namespace='tb3_0',
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[tb3_0_recovery_yaml],
            output='screen'
            ),
        # tb3_1
        Node(
            namespace='tb3_1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[tb3_1_planner_yaml]
            ),
        Node(
            namespace='tb3_1',
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[tb3_1_controller_yaml]
            ),
        Node(
            namespace='tb3_1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[tb3_1_bt_navigator_yaml]
            ),
        Node(
            namespace='tb3_1',
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[tb3_1_recovery_yaml],
            output='screen'
            ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_path_planner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        # tb3_0
                                        'tb3_0/planner_server', 
                                        'tb3_0/controller_server', 
                                        'tb3_0/bt_navigator',
                                        'tb3_0/recoveries_server',
                                        # tb3_1
                                        'tb3_1/planner_server', 
                                        'tb3_1/controller_server', 
                                        'tb3_1/bt_navigator',
                                        'tb3_1/recoveries_server',]}])
    ])
