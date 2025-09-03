#!/bin/bash
set -e

# Sourcing ROS 2 Humble setup
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "sourcing /opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "notfound /opt/ros/$ROS_DISTRO/setup.bash"
    echo "sourcing /opt/ros/$ROS_DISTRO/install/setup.bash"
    source /opt/ros/$ROS_DISTRO/install/setup.bash
fi

# Sourcing the local workspace setup
echo "sourcing $WORKSPACE_ROOT/install/setup.bash"
source "$WORKSPACE_ROOT/install/setup.bash"

# Execute the command passed to the container
exec "$@"
