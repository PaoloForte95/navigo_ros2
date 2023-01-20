# Copyright (c) 2022 Paolo Forte
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,GroupAction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    wheel_loader_launch_dir = get_package_share_directory('wheel_loader_launch')
    wheel_loader_gazebo_dir = get_package_share_directory('wheel_loader_gazebo')
    description_dir = get_package_share_directory('wheel_loader_description')
    maps_dir = get_package_share_directory('orunav2_maps')
    worlds_dir = get_package_share_directory('gazebo_worlds_oru')
  
    # Create the launch configuration variables
    

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')

    robots = [
        {'name': 'robot1', 'x_pose': '1.0', 'y_pose': '0.0', 'z_pose': '0.4',
                           'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0'}]
  
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('cmd_vel', 'controller/cmd_vel')
                  ]


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(description_dir, 'launch', 'wheel_loader_namespaced.rviz'),
        description='Full path to the RVIZ config file to use')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(worlds_dir, 'worlds', 'empty.world'),
        description='Full path to world model file to load')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            maps_dir, 'maps', 'empty.yaml'),
        description='Full path to map file to load')

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(wheel_loader_launch_dir, 'params', 'L70_navigation_configs.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

        
    # Start Algoryx
    start_algoryx_cmd = ExecuteProcess(
        cmd=['python3', 'src/algoryx/examples/my_wheel_loader_terrain_ros2.py'],
        output='screen')

    spawn_robots_cmds = []
    for robot in robots:
         spawn_robots_cmds.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(wheel_loader_gazebo_dir, 'launch', 'wheel_loader_empty.py')),
        launch_arguments={'namespace': robot['name'],
                          'robot_name': robot['name'],
                          'x_pose': robot['x_pose'],
                          'y_pose': robot['y_pose'],
                          'yaw': robot['yaw'],
                          'world':world,
                          'use_gui': 'False',
                          }.items()))

    default_nav_to_pose_bt_xml = os.path.join(
            get_package_share_directory('plan2_behavior_tree'),
            'behavior_trees', 'planning_w_navigation.xml')

    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot['name'] + '_params_file')
        group = GroupAction([

         Node(
                    name= robot['name']+'_origin_broadcaster',
                    package='tf2_ros',
                    namespace = robot['name'],
                    executable='static_transform_publisher',
                    remappings=remappings,
                    arguments = ['--frame-id','world', '--child-frame-id', 'map']),
        
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                            os.path.join(description_dir, 'launch', 'wheel_loader_rviz.py')),
                    launch_arguments={
                                    'namespace': robot['name'],
                                    'use_namespace': 'True',
                                    'use_sim_time': 'True',
                                    'use_robot_state_pub': 'False',
                                    'use_joint_state_publisher_gui': 'False',
                                    'rviz_config': rviz_config_file}.items()),

            
            IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(wheel_loader_launch_dir, 'launch', 'wheel_loader_navigation.py')),
        launch_arguments={'namespace': robot['name'],
                          'robot_name': robot['name'],
                          'use_namespace': 'True',
                           'use_simulator': 'False',
                           'use_sim_time': 'True',
                           'use_rviz': 'False',
                           'headless': 'False',
                           'autostart': 'True',
                           'rviz_config_file': rviz_config_file,
                           'map': map_yaml_file,
                           'params_file' : params_file,
                           'use_robot_state_pub': 'False',
                           'controller_prefix': 'controller/',
                            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                           'use_composition': 'False',
                           'use_selector': 'True'
                          }.items())
        
        ])
        nav_instances_cmds.append(group)


    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    
    ld.add_action(start_algoryx_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)
    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)



    return ld
