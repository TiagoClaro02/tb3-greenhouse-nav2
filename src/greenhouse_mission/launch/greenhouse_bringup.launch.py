#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_greenhouse = get_package_share_directory('greenhouse_mission')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    world_path = os.path.join(pkg_greenhouse, 'worlds', 'greenhouse.sdf')
    map_yaml_path = os.path.join(pkg_greenhouse, 'maps', 'greenhouse_map.yaml')
    nav2_params_path = os.path.join(pkg_greenhouse, 'config', 'nav2_params.yaml')
    waypoints_path = os.path.join(pkg_greenhouse, 'config', 'waypoints.yaml')

    with open(waypoints_path) as f:
        waypoints_data = yaml.safe_load(f)
    default_spawn_x = str(waypoints_data['spawn_x'])
    default_spawn_y = str(waypoints_data['spawn_y'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value=default_spawn_x,
        description='Robot spawn x position (from waypoints.yaml spawn_x)')

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value=default_spawn_y,
        description='Robot spawn y position (from waypoints.yaml spawn_y)')

    # --- Gazebo server ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    # --- Gazebo client ---
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    # --- Robot state publisher (reused directly from turtlebot3_gazebo) ---
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Spawn + ros_gz_bridge (reused directly from turtlebot3_gazebo) ---
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    # TurtleBot3 model meshes live under turtlebot3_gazebo/models -- required
    # for spawn to find geometry, same as the stock empty_world.launch.py does.
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_tb3_gazebo, 'models'))

    # --- Nav2, using our static map + our (to-be-tuned) params ---
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': nav2_params_path,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # bringup_launch.py deliberately does NOT start RViz -- Nav2 treats it as
    # a separate concern, launched via this file.
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
