# This message acts as a container for all possible command arguments.
# Only the message corresponding to the 'command' field in the action goal 
# should be populated. The others will be ignored by the server.

# Standard ROS 2 header for timestamp and frame info
std_msgs/Header header

# Command arguments
ArmMoveOptions arm_goal
NavOptions nav_goal
WaitDuration wait_goal