# ID of the robot
int32 robot_id
#####################

builtin_interfaces/Time stamp

# Operation State unknown 
int32 OPERATION_UNKNOWN = 1
# Start operation not performed
int32 START_OPERATION_NOT_PERFORMED = 2
# Goal operation not performed
int32 GOAL_OPERATION_NOT_PERFORMED = 3
# Start operation performed
int32 START_OPERATION_PERFORMED = 4
# Goal operation performed
int32 GOAL_OPERATION_PERFORMED = 5
# Start/Goal operation failed
int32 OPERATION_FAILURE = 6

# Status of the fork controller.
int32 status
