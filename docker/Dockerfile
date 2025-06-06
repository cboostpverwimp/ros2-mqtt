FROM ros:jazzy-ros-base

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    wget \
    xauth \
    python3-pip \
    x11-xserver-utils \
    && apt install -y \
    software-properties-common \
    ros-${ROS_DISTRO}-mqtt-client \
    '~nros-${ROS_DISTRO}-rqt*' \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
ENV SHELL=/bin/bash
RUN /bin/bash -c "source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; colcon mixin update default"

# Create workspace so that user own this directory
RUN mkdir -p /app/src
WORKDIR /app
COPY src/. src/.
RUN sudo apt update && rosdep update \
    && rosdep install --from-paths src --ignore-src -y -r

# Colcon build workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"

# Copy configuration files
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc \
    && echo 'source /app/install/setup.bash' >> ~/.bashrc

# Set entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]