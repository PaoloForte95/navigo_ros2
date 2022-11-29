# If no operation should be done
int32 NO_OPERATION = 1

# If cargo should be unloaded (e.g. lowering the forks)
int32 UNLOAD = 2

# If cargo should be loadeded (e.g. lifting the forks)
int32 LOAD = 3

# If the cargo location should be detected using perception (for load action)
int32 LOAD_DETECT = 4

# If the fork should be used to lower the support legs
int32 ACTIVATE_SUPPORT_LEGS = 5

# Not sure what this means - TODO
int32 LOAD_DETECT_ACTIVE = 6

# Pick all items in the mission specification
int32 PICK_ITEMS = 7

# Unwrap a block
int32 UNWRAP_BLOCK = 8

# Unstack material
int32 UNSTACK_MATERIAL = 9

# Stack material
int32 STACK_MATERIAL = 10

# If cargo should be loadeded at a higher position (i.e. lifting the forks or bucket to a known high)
int32 APPROACH_MATERIAL = 11

# Move the fork down for handling the laod safetly
int32 HANDLE_LOAD = 12

# If the cargo location should be detected using perception (for push action)
int32 PUSH_DETECT = 13

# Push  material 
int32 PUSH_MATERIAL = 14



int32 operation
int32 property1
int32 property2
orunav_msgs/IliadItemArray itemlist
