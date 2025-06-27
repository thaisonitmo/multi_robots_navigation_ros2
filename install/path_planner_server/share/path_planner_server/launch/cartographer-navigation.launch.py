from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
    # map_file = \
    #     os.path.join(get_package_share_directory('map_server'), \
    #     'config', \
    #     'turtlebot_area.yaml')
    # amcl_yaml = \
    #     os.path.join(get_package_share_directory('localization_server'), \
    #     'config', \
    #     'amcl_config_initialized.yaml')

    ld = LaunchDescription()

    planner_server_cmd = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
            )
    controller_cmd = Node(
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml]
            )
    bt_navigator_cmd = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
            )
    recovery_cmd = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'
            )
    life_cycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': [
                                        # 'map_server',
                                        # 'amcl',
                                        'planner_server', 
                                        'controller_server', 
                                        'bt_navigator',
                                        'recoveries_server']}])

    cartographer_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cartographer_slam'),
                        'launch/cartographer.launch.py')))

    ld.add_action(cartographer_launch_file)
    ld.add_entity(planner_server_cmd)
    ld.add_entity(controller_cmd)
    ld.add_entity(bt_navigator_cmd)
    ld.add_entity(recovery_cmd)
    ld.add_entity(life_cycle_manager_cmd)


    return ld
