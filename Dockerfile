# Stage 1: Build the custom planner
FROM ros:humble AS builder

# Set non-interactive frontend to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for TurtleBot4 simulation, Nav2, SLAM, and building
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a colcon workspace
WORKDIR /ros2_ws

# Copy your custom planner source code into the workspace
# Assumes your planner is in a directory named 'src' next to the Dockerfile
COPY ./src ./src

# Source ROS and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install


# Stage 2: Create the final, lean image
FROM ros:humble

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install only the runtime dependencies needed
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Copy the built workspace from the builder stage
WORKDIR /ros2_ws
COPY --from=builder /ros2_ws .

# Set up the entrypoint to source ROS and the workspace
CMD ["bash"]