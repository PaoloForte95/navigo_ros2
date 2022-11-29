# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node





def generate_launch_description():
    
    namespace = LaunchConfiguration('namespace')
    pose = {'x': LaunchConfiguration('x_pose', default='0'),
                'y': LaunchConfiguration('y_pose', default='0'),
                'z': LaunchConfiguration('z_pose', default='0.1'),
                'R': LaunchConfiguration('roll', default='0.00'),
                'P': LaunchConfiguration('pitch', default='0.00'),
                'Y': LaunchConfiguration('yaw', default='0.00')}
    
    box_name = LaunchConfiguration('box_name')
    box_sdf = LaunchConfiguration('box_sdf')
    

    my_bringup_dir = get_package_share_directory('construction_materials')
    
    
    declare_box_name_cmd = DeclareLaunchArgument(
        'box_name',
        default_value='box',
        description='name of the box')

    declare_box_sdf_cmd = DeclareLaunchArgument(
        'box_sdf',
        default_value=os.path.join(my_bringup_dir, 'urdf', 'box2.model'),
        description='Full path to box sdf file to spawn the box in gazebo')


    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')



    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', box_name,
            '-file', box_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])



    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_box_name_cmd)
    ld.add_action(declare_box_sdf_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    

    return ld