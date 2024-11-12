import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the path to the package where your map file is stored
    package_launch_name = 'navigo2_launch'
    maps_package = 'navigo2_maps'
    map_file_path = FindPackageShare(maps_package).find(maps_package) + '/maps/narnia_segmented.yaml'
    params_file_path = FindPackageShare(package_launch_name).find(package_launch_name) + '/params/global_planner_params.yaml'
    rviz_config_file = FindPackageShare(package_launch_name).find(package_launch_name) + '/rviz/navigation_view.rviz'
    
    robot = [{'name': 'robot1', 'x_pose': '0.0', 'y_pose': '-2.0', 'z_pose': '0.4',
     'roll': '0.0', 'pitch': '0.0', 'yaw': '1.57'}]
    
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
    )

    # Static transform publisher to make `odom` the same as `map`
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    # Static transform publisher for `base_link` the same as `odom` (this will give the start pose)
    # Use the following or the initial_pose_to_tf_node.
    # static_transform_publisher2 = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_transform_publisher_map_to_odom',
    #    output='screen',
    #    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )
    
    initial_pose_to_tf_node = Node(
            package='ecceleron_global_planner',
            executable='initial_pose_to_tf_node',
            name='initial_pose_to_tf_node',
            output='screen',
            parameters=[
                {'parent_frame_id': 'map'},  # Default parent frame ID
                {'child_frame_id': 'base_link'},  # Default child frame ID
            ]
        )

    bt_xml = os.path.join(get_package_share_directory('navigo2_behavior_tree'),
            'behavior_trees', 'plan_to_pose.xml')

    # Include the bringup launch file
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(package_launch_name), '/launch/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_file_path,
            'params_file': params_file_path,
            'only_path_planning': 'True',
            'bt_xml': bt_xml,
        }.items()
        
    )

    return LaunchDescription([
        rviz,
        static_transform_publisher,
        #static_transform_publisher2,
        initial_pose_to_tf_node,
        bringup_launch,
    ])
