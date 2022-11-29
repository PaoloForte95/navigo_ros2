cmake_minimum_required(VERSION 3.8)
project(orunav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)


set(msg_files
   "msg/ActivePolygonConstraints.msg"   
   "msg/ControllerTrajectoryChunkVec.msg"  
   "msg/EnvelopeID.msg"        
   "msg/Path.msg"     
   "msg/PathParameters.msg"           
   "msg/TimeEnvelope.msg"
   "msg/ActiveRobots.msg"               
   "msg/ControllerTrajectoryStep.msg"      
   "msg/ForkCommand.msg"       
   "msg/PolygonConstraint.msg"    
   "msg/TimeInterval.msg"
   "msg/BrakeCommand.msg"               
   "msg/ControllerTrajectoryValue.msg"     
   "msg/ForkLoadHandling.msg"  
   "msg/PoseSteering.msg"         
   "msg/Trajectories.msg"
   "msg/ControllerCommand.msg"          
   "msg/ControllerVelocities.msg"          
   "msg/ForkReport.msg"        
   "msg/RobotConstraints.msg"    
   "msg/Trajectory.msg"
   "msg/ControllerConstraints.msg"      
   "msg/CoordinatorTime.msg"               
   "msg/ForkState.msg"         
   "msg/RobotEnvelopes.msg"       
   "msg/TrajectoryPoint.msg"
   "msg/ControllerReport.msg"           
   "msg/CoordinatorTimeVec.msg"           
   "msg/Load.msg"              
   "msg/RobotPathReplanning.msg"  
   "msg/TwistSteering.msg"
   "msg/ControllerState.msg"            
   "msg/DeltaT.msg"                        
   "msg/Operation.msg"         
   "msg/RobotReschedule.msg"
   "msg/ControllerTrajectoryChunk.msg" 
   "msg/DeltaTVec.msg"                     
   "msg/ParkingPolygons.msg"  
   "msg/RobotTarget.msg"
   "msg/Task.msg"
   "msg/RobotReport.msg"
   "msg/Shape.msg"
   "msg/LevelCommand.msg"
   "msg/JackReport.msg"
   "msg/JackState.msg"
   "msg/ObjectPose.msg"
   "msg/ObjectType.msg"
   "msg/VectorMap.msg"
   "msg/GeoFence.msg"
   "msg/DrillTarget.msg"
   "msg/DrillTargetArray.msg"
   "msg/IliadItem.msg"
   "msg/IliadItemArray.msg"
   "msg/EBrake.msg"
   "msg/ComputeTaskStatus.msg"
   "msg/ReplanStatus.msg"
   "msg/BucketCommand.msg"
   "msg/BucketReport.msg"
   "msg/BucketState.msg"
   "msg/OperationReport.msg"
)


set(srv_files
  "srv/BrakeTask.srv"
  "srv/GetPath.srv"
  "srv/GetParametrizedPath.srv"
  "srv/SetTask.srv"
  "srv/ComputeTask.srv"
  "srv/ExecuteTask.srv"
  "srv/UpdateTask.srv"
  "srv/GetPolygonConstraints.srv"
  "srv/GetSmoothedPath.srv"
  "srv/GetSmoothedStraightPath.srv"
  "srv/GetDeltaTVec.srv"
  "srv/ObjectPoseEstimation.srv"
  "srv/GetDrillTargets.srv"
  "srv/GetGeoFence.srv"
  "srv/GetVectorMap.srv"
  "srv/GetPolygonFootPrint.srv"
  "srv/SetDelay.srv"
  "srv/SetPenetrationRate.srv"
  "srv/SetBool.srv"
  "srv/Trigger.srv"
   # etc
  )


rosidl_generate_interfaces(${PROJECT_NAME} 
   "msg/ActivePolygonConstraints.msg"   
   "msg/ControllerTrajectoryChunkVec.msg"  
   "msg/EnvelopeID.msg"        
   "msg/Path.msg"     
   "msg/PathParameters.msg"           
   "msg/TimeEnvelope.msg"
   "msg/ActiveRobots.msg"               
   "msg/ControllerTrajectoryStep.msg"      
   "msg/ForkCommand.msg"       
   "msg/PolygonConstraint.msg"    
   "msg/TimeInterval.msg"
   "msg/BrakeCommand.msg"               
   "msg/ControllerTrajectoryValue.msg"     
   "msg/ForkLoadHandling.msg"  
   "msg/PoseSteering.msg"         
   "msg/Trajectories.msg"
   "msg/ControllerCommand.msg"          
   "msg/ControllerVelocities.msg"          
   "msg/ForkReport.msg"        
   "msg/RobotConstraints.msg"    
   "msg/Trajectory.msg"
   "msg/ControllerConstraints.msg"      
   "msg/CoordinatorTime.msg"               
   "msg/ForkState.msg"         
   "msg/RobotEnvelopes.msg"       
   "msg/TrajectoryPoint.msg"
   "msg/ControllerReport.msg"           
   "msg/CoordinatorTimeVec.msg"           
   "msg/Load.msg"              
   "msg/RobotPathReplanning.msg"  
   "msg/TwistSteering.msg"
   "msg/ControllerState.msg"            
   "msg/DeltaT.msg"                        
   "msg/Operation.msg"         
   "msg/RobotReschedule.msg"
   "msg/ControllerTrajectoryChunk.msg" 
   "msg/DeltaTVec.msg"                     
   "msg/ParkingPolygons.msg"  
   "msg/RobotTarget.msg"
   "msg/Task.msg"
   "msg/RobotReport.msg"
   "msg/Shape.msg"
   "msg/LevelCommand.msg"
   "msg/JackReport.msg"
   "msg/JackState.msg"
   "msg/ObjectPose.msg"
   "msg/ObjectType.msg"
   "msg/VectorMap.msg"
   "msg/GeoFence.msg"
   "msg/DrillTarget.msg"
   "msg/DrillTargetArray.msg"
   "msg/IliadItem.msg"
   "msg/IliadItemArray.msg"
   "msg/EBrake.msg"
   "msg/ComputeTaskStatus.msg"
   "msg/ReplanStatus.msg"
   "msg/BucketCommand.msg"
   "msg/BucketReport.msg"
   "msg/BucketState.msg"
   "msg/OperationReport.msg"
   
   
   #Services
   "srv/BrakeTask.srv"
  "srv/GetPath.srv"
  "srv/GetParametrizedPath.srv"
  "srv/SetTask.srv"
  "srv/ComputeTask.srv"
  "srv/ExecuteTask.srv"
  "srv/UpdateTask.srv"
  "srv/GetPolygonConstraints.srv"
  "srv/GetSmoothedPath.srv"
  "srv/GetSmoothedStraightPath.srv"
  "srv/GetDeltaTVec.srv"
  "srv/ObjectPoseEstimation.srv"
  "srv/GetDrillTargets.srv"
  "srv/GetGeoFence.srv"
  "srv/GetVectorMap.srv"
  "srv/GetPolygonFootPrint.srv"
  "srv/SetDelay.srv"
  "srv/SetPenetrationRate.srv"
  "srv/SetBool.srv"
  "srv/Trigger.srv"
   
   
  #${msg_files}
  #${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)


install(
  FILES my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

#ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
