# Use ROS 2 Humble Desktop as the base image
FROM osrf/ros:humble-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    sudo \
    wget \
    curl \
    tmux \
    ruby \
    tmuxinator

# Install PX4
RUN cd /root && \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot && \
    make px4_sitl

# Setup Micro XRCE-DDS Agent & Client
RUN cd /root && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/

# Build ROS 2 Workspace ws_sensor_combined
RUN mkdir -p /root/ws_sensor_combined/src && \
    cd /root/ws_sensor_combined/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/ws_sensor_combined && colcon build"

# Build ROS 2 Workspace ws_offboard_control
RUN mkdir -p /root/ws_offboard_control/src && \
    cd /root/ws_offboard_control/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/ws_offboard_control && colcon build"

# Install Python requirements. If you don't have gpu, uncomment next line -torch cpu installation-
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install \
    mavsdk \
    aioconsole \
    pygame \
    opencv-python \
    ultralytics

RUN apt-get install -y ros-humble-ros-gzgarden

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 uninstall -y numpy

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY . /root/PX4-ROS2-Gazebo-YOLOv8
COPY models/. /root/.gz/models/
COPY models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY worlds/default_docker.sdf /root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

# Setup gimbal joints for camera control
RUN python3 /root/PX4-ROS2-Gazebo-YOLOv8/setup_gimbal.py

# Additional Configs
RUN echo "source /root/ws_sensor_combined/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc

# Copy tmuxinator configuration
COPY px4_ros2_gazebo.yml /root/.config/tmuxinator/px4_ros2_gazebo.yml

# Set up tmuxinator
RUN echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc

# Set default command to start tmuxinator
CMD ["tmuxinator", "start", "px4_ros2_gazebo"]
