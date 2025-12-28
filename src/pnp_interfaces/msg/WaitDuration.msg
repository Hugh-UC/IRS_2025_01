# Standard ROS 2 header for timestamp and frame info
std_msgs/Header header

# Arguments for the "wait_goal" command
# If duration = 0.0:
#     - For _wait_at_position/_wait_at_waypoint: 'skip wait'
#     - For _set_wait_seconds: self._wait_seconds = 0.0
# If duration = -1.0:
#     - For _wait_at_position/_wait_at_waypoint: 'use class default' (self._wait_seconds)
#     - For _set_wait_seconds: 'ignore and log warning'
float64 duration  # in seconds: e.g. 3.0 second(s)