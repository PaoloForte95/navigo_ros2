import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    # Find the path to the package where your map file is stored
    package_name = 'navigo2_launch'
    maps_package = 'navigo2_maps'
    a25_gazebo_dir = get_package_share_directory('a25_gazebo')
    description_dir = get_package_share_directory('a25_description')
    worlds_dir = get_package_share_directory('ecceleron_gazebo_worlds')
    map_file_path = FindPackageShare(maps_package).find(maps_package) + '/maps/mine_small_reverse.yaml'
    params_file_path = FindPackageShare(package_name).find(package_name) + '/param/navigation_params.yaml'
    rviz_config_file = FindPackageShare(package_name).find(package_name) + '/rviz/navigation_view.rviz'

    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(worlds_dir, 'worlds', 'mine_small_reverse.world'),
        description='Full path to world model file to load')

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
    )
    
    robot = [{'name': 'a25', 'x_pose': '0.0', 'y_pose': '-2.0', 'z_pose': '0.4',
                           'roll': '0.0', 'pitch': '0.0', 'yaw': '1.57'}]
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
    static_transform_publisher2 = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='static_transform_publisher_map_to_odom',
       output='screen',
       arguments=[robot['x_pose'], robot['y_pose'], '0', '0', '0', robot['yaw'], 'odom', 'base_link']
    )

    spawn_robots_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(a25_gazebo_dir, 'launch', 'a25_empty.py')),
        launch_arguments={'robot_name': robot['name'],
                          'x_pose': robot['x_pose'],
                          'y_pose': robot['y_pose'],
                          'yaw': robot['yaw'],
                          'world':world,
                          'use_gui': 'False',
                          }.items())
    
    rviz_launch_cmd = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                            os.path.join(description_dir, 'launch', 'a25_rviz.py')),
                    launch_arguments={
                                    'namespace': robot['name'],
                                    'use_namespace': 'True',
                                    'use_robot_state_pub': 'False',
                                    'use_sim_time': 'True',
                                    'use_joint_state_publisher_gui': 'False',
                                    'rviz_config': rviz_config_file}.items())

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
            [FindPackageShare(package_name), '/launch/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_file_path,
            'params_file': params_file_path,
            'bt_xml': bt_xml,
        }.items()
        
    )

    return LaunchDescription([
        rviz,
        static_transform_publisher,
        static_transform_publisher2,
        initial_pose_to_tf_node,
        bringup_launch,
        spawn_robots_cmd,
        rviz_launch_cmd
    ])
