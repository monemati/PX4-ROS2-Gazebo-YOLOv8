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
    python3-dev \
    python3-setuptools \
    python3-lxml \
    python3-matplotlib \
    python3-pyparsing \
    python3-pexpect \
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
    rapidjson-dev \
    sudo \
    wget \
    curl \
    tmux \
    ruby \
    tmuxinator

# Add Gazebo Garden repository and install Gazebo + ROS-GZ bridge
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
    libgz-sim7-dev \
    ros-humble-ros-gzgarden

# Install ArduPilot prerequisites (the install-prereqs script refuses root, so install manually)
RUN apt-get update && apt-get install -y \
    python3-future \
    python3-serial \
    python3-wxgtk4.0 \
    python3-opencv \
    python3-empy \
    python3-yaml \
    ccache \
    gawk && \
    pip3 install MAVProxy pymavlink future lxml

# Clone and build ArduPilot
RUN cd /root && \
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    ./waf configure --board sitl && \
    ./waf copter

# Build ardupilot_gazebo plugin (GZ_VERSION=garden for gz-sim7)
ENV GZ_VERSION=garden
RUN cd /root && \
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
    cd ardupilot_gazebo && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j$(nproc)

# Install Python requirements. If you don't have gpu, uncomment next line -torch cpu installation-
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install \
    mavsdk \
    pymavlink \
    aioconsole \
    pygame \
    opencv-python \
    ultralytics

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 uninstall -y numpy

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY . /root/PX4-ROS2-Gazebo-YOLOv8
COPY models/. /root/.gz/models/
COPY models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/

# Copy ArduPilot world
COPY worlds/ardupilot_default.sdf /root/ardupilot_gazebo/worlds/ardupilot_default.sdf

# Configure gimbal camera (add explicit topic, remove unavailable plugins)
RUN python3 /root/PX4-ROS2-Gazebo-YOLOv8/setup_gimbal_ardupilot.py

# Environment configuration
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models:/root/ardupilot_gazebo/models:/root/ardupilot_gazebo/worlds" >> /root/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/ardupilot_gazebo/build" >> /root/.bashrc && \
    echo "export PATH=\$PATH:/root/ardupilot/Tools/autotest:/root/.local/bin" >> /root/.bashrc

# Copy tmuxinator configuration
COPY ardupilot_ros2_gazebo.yml /root/.config/tmuxinator/ardupilot_ros2_gazebo.yml

# Set default command to start tmuxinator
CMD ["tmuxinator", "start", "ardupilot_ros2_gazebo"]
