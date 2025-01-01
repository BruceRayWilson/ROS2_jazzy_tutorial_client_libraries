# Use ROS 2 Jazzy base image
FROM ros:jazzy-perception

# Create non-root user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-gazebo-* \
    nano \
    && rm -rf /var/lib/apt/lists/* \
    && chown -R $USERNAME:$USERNAME /ros2_diff_drive_ws

# Set working directory
WORKDIR /ros2_diff_drive_ws

# Create an entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && chown $USERNAME:$USERNAME /entrypoint.sh

# Switch to non-root user
USER $USERNAME

# Use the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]