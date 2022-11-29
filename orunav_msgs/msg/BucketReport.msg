# ID of the robot
int32 robot_id
#####################

builtin_interfaces/Time stamp

# Bucket is in known position given by the height value
int32 BUCKET_POSITION_KNOWN = 0
# Bucket is in unknown position
int32 BUCKET_POSITION_UNKNOWN = 1
# Bucket are in 'low' position - we can move in below a block of material from here
int32 BUCKET_POSITION_LOW = 2
# Bucket is in a 'high' position (e.g. for a wheel loader).
int32 BUCKET_POSITION_HIGH = 3
# Moving up
int32 BUCKET_MOVING_UP = 4
# Moving down
int32 BUCKET_MOVING_DOWN = 5
# Failure
int32 BUCKET_FAILURE = 6
# Bucket is in a 'higher' postion - for unstack or stack operations
int32 BUCKET_POSITION_STACK = 7
# Bucket tilt is in a 'high' position
int32 BUCKET_TILT_LOW = 8
# Bucket tilt is in a 'high' position
int32 BUCKET_TILT_HIGH = 9



# Status of the fork controller.
int32 status_elevation

# Status of the fork controller.
int32 status_tilt

#####################

# The current state
BucketState state

#####################
