# ------------------------------------------------------------------------------------------------------------------------
# Base stage: Build workspace with required ROS 2 packages, MoveIt, and Nav2
# ------------------------------------------------------------------------------------------------------------------------
FROM ros:humble-ros-base-jammy AS base

# Environment
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE_ROOT=/lab_shell_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-dev \
    build-essential \
    curl \
    lsb-release \
    wget \
    gnupg2 \
    qtdeclarative5-dev \
    libqt5widgets5 \
    ros-humble-moveit \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-vision-opencv \
    ros-humble-rviz2 \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-nav2-simple-commander \
    ros-humble-tf2-tools \
    libx11-dev \
    libxext-dev \
    libxtst-dev \
    libxrender-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create a ROS 2 workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p src

# Copy your local ROS 2 package into the container
COPY ./src ${WORKSPACE_ROOT}/src

# Source ROS setup and build workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Clean up
RUN rm -rf ${WORKSPACE_ROOT}/src \
           ${WORKSPACE_ROOT}/log \
           ${WORKSPACE_ROOT}/build \
           /tmp/* \
           /var/lib/apt/lists/*

# ------------------------------------------------------------------------------------------------------------------------
# Final stage: Minimal runtime image with built workspace and GUI support
# ------------------------------------------------------------------------------------------------------------------------
FROM ros:humble-ros-base-jammy AS final

ENV WORKSPACE_ROOT=/lab_shell_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

WORKDIR /

# Copy built workspace from base stage
COPY --from=base /lab_shell_ws /lab_shell_ws

# Optional: custom entrypoint (make sure this script exists)
COPY workspace_entrypoint.sh /workspace_entrypoint.sh
RUN chmod +x /workspace_entrypoint.sh

WORKDIR ${WORKSPACE_ROOT}

# Entry point
ENTRYPOINT ["/workspace_entrypoint.sh"]
CMD ["bash"]
