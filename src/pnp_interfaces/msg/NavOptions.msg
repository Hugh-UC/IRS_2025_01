# Standard ROS 2 header for timestamp and frame info
std_msgs/Header header

# Arguments for the "nav_goal" command
string target_waypoint      # e.g. "home", "conveyor", "drop-off"
float32 target_spin         # e.g. 6.283 (360 deg)