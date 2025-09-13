# Stage 1: Build the custom planner
FROM ros:humble AS builder

# Set non-interactive frontend to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists first (this step will be cached)
RUN apt-get update

# Install dependencies for TurtleBot4 simulation, Nav2, SLAM, and building
RUN apt-get install -y --no-install-recommends \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a colcon workspace
WORKDIR /ros2_ws
COPY ./src ./src

# Source ROS and build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Stage 2: Create the final, lean image
FROM ros:humble

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists first (this step will be cached)
RUN apt-get update

# Install only the runtime dependencies needed
RUN apt-get install -y --no-install-recommends \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    x11-apps \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Copy the built workspace from the builder stage
WORKDIR /ros2_ws
COPY --from=builder /ros2_ws .

# Copy and prepare the launch script
COPY ./launch_all.sh .
RUN chmod +x ./launch_all.sh

# Set up the entrypoint to run the launch script by default
CMD ["./launch_all.sh"]