import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,TextSubstitution,PythonExpression


def generate_launch_description():
  # These are the arguments you can pass this launch file, for example paused:=True -->
  paused = LaunchConfiguration('paused')
  use_sim_time = LaunchConfiguration('use_sim_time')
  extra_gazebo_args = LaunchConfiguration('extra_gazebo_args')
  gui = LaunchConfiguration('gui')
  recording = LaunchConfiguration('recording')
  headless = LaunchConfiguration('headless')
  debug = LaunchConfiguration('debug')
  physics = LaunchConfiguration('physics')
  verbose = LaunchConfiguration('verbose')
  output = LaunchConfiguration('output')
  world_file = LaunchConfiguration('world')
  respawn_gazebo = LaunchConfiguration('respawn_gazebo')
  use_clock_frequency = LaunchConfiguration('use_clock_frequency')
  pub_clock_frequency = LaunchConfiguration('pub_clock_frequency')
  enable_ros_network = LaunchConfiguration('enable_ros_network')
  server_required = LaunchConfiguration('server_required')
  gui_required = LaunchConfiguration('gui_required')


  pkg_dir = get_package_share_directory('navthon_maps')
  worlds_pkg_dir = get_package_share_directory('navthon_maps')
  launch_dir = os.path.join(pkg_dir, 'launch')

  declare_paused_cmd = DeclareLaunchArgument(
      'paused',
      default_value='False')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if True')
  
  declare_extra_gazebo_args_cmd = DeclareLaunchArgument(
      'extra_gazebo_args',
      default_value='')

  declare_gui_cmd = DeclareLaunchArgument(
      'gui',
      default_value='True',
      description='Whether to start the simulator')

  declare_recording_cmd = DeclareLaunchArgument(
      'recording',
      default_value='False')
  
  declare_headless_cmd = DeclareLaunchArgument(
      'headless',
      default_value='False')
  
  declare_debug_cmd = DeclareLaunchArgument(
      'debug',
      default_value='False')

  declare_physics_cmd = DeclareLaunchArgument(
      'physics',
      default_value='ode')
  
  declare_verbose_cmd = DeclareLaunchArgument(
      'verbose',
      default_value='False')

  declare_output_cmd = DeclareLaunchArgument(
      'output',
      default_value='screen')

  declare_world_name_cmd = DeclareLaunchArgument(
      'world',
      default_value=os.path.join(worlds_pkg_dir, 'worlds', 'empty.world'),
      description='Full path to world model file to load')
  
  declare_respawn_gazebo_cmd = DeclareLaunchArgument(
      'respawn_gazebo',
      default_value='False')
  
  declare_use_clock_frequency_cmd = DeclareLaunchArgument(
      'use_clock_frequency',
      default_value='False')
  
  declare_pub_clock_frequency_cmd = DeclareLaunchArgument(
      'pub_clock_frequency',
      default_value='100')

  declare_enable_ros_network_cmd = DeclareLaunchArgument(
      'enable_ros_network',
      default_value='True')

  declare_server_required_cmd = DeclareLaunchArgument(
      'server_required',
      default_value='False')
  
  declare_gui_required_cmd = DeclareLaunchArgument(
      'gui_required',
      default_value='False')
  

  launch_ros.actions.SetParameter(name='use_sim_time', value = use_sim_time)
  launch_ros.actions.SetParameter(name='gazebo/pub_clock_frequency', value = pub_clock_frequency)
  launch_ros.actions.SetParameter(name='gazebo/enable_ros_network', value = enable_ros_network) 

  start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so','-s','libgazebo_ros_factory.so', world_file , '-e' , physics , extra_gazebo_args],
        cwd=[launch_dir], output='screen', respawn='False')

  start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([gui, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen', respawn='False')

  # Create the launch description and populate
  ld = LaunchDescription()
  ld.add_action(declare_paused_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_extra_gazebo_args_cmd)
  ld.add_action(declare_gui_cmd)
  ld.add_action(declare_recording_cmd)
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_debug_cmd)
  ld.add_action(declare_physics_cmd)
  ld.add_action(declare_verbose_cmd)
  ld.add_action(declare_output_cmd)
  ld.add_action(declare_world_name_cmd)
  ld.add_action(declare_respawn_gazebo_cmd)
  ld.add_action(declare_use_clock_frequency_cmd)
  ld.add_action(declare_pub_clock_frequency_cmd)
  ld.add_action(declare_enable_ros_network_cmd)
  ld.add_action(declare_server_required_cmd)
  ld.add_action(declare_gui_required_cmd)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)

  

  return ld