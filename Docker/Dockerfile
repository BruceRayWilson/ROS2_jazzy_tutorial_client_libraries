# Use ROS 2 Jazzy base image
FROM ros:jazzy-perception

# Set environment variables for better maintainability
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Create non-root user with arguments for username, UID, and GID
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

# Create environment variables that persist
ENV USERNAME=${USERNAME}
ENV USER_UID=${USER_UID}
ENV USER_GID=${USER_GID}

# Set up workspace directory
WORKDIR /ros2_ws

# Copy and set up entrypoint script
COPY entrypoint.sh /entrypoint.sh

RUN chown -R $USER_UID:$USER_GID /ros2_ws /entrypoint.sh

# Ensure the script is executable
RUN chmod +x /entrypoint.sh

# Create user and group with error handling
RUN groupadd --gid $USER_GID $USERNAME || \
    if [ $? -eq 9 ]; then \
        groupmod -g $USER_GID $USERNAME || true; \
    fi \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME || \
    if [ $? -eq 9 ]; then \
        usermod -u $USER_UID -g $USER_GID $USERNAME || true; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        python3-pip \
        python3-colcon-common-extensions \
        ros-jazzy-gazebo-* \
        nano \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Switch to non-root user
USER $USERNAME

# Set up environment variables for the user
ENV HOME=/home/$USERNAME
ENV SHELL=/bin/bash

# Use the entrypoint script
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]