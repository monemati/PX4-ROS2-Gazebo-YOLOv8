# PX4-ROS2-Gazebo-YOLOv8
Aerial Object Detection using a Drone with PX4 Autopilot and ROS 2. PX4 SITL and Gazebo Garden used for Simulation. YOLOv8 used for Object Detection.

## Features
- Keyboard-controlled drone flight (WASD + arrow keys) via MAVSDK
- 2-axis gimbal camera control (pitch and yaw) adjustable during flight
- YOLOv8 real-time object detection with resizable display window
- Moving car target in the simulation for tracking demonstrations
- All services orchestrated via tmuxinator in a single tiled-pane window
- Docker-based setup with GPU passthrough and X11 forwarding

## Demo
https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/assets/58460889/fab19f49-0be6-43ea-a4e4-8e9bc8d59af9

## Docker
- You can pull the image (already built) or use the provided Dockerfile.

### Prerequisites
Allow Docker to access the X11 display:
```commandline
xhost +local:docker
```

### Pull The Image
```commandline
# Already built and uploaded in dockerhub; You can skip this step, if you want to build your own custom image.
docker pull monemati/px4_ros2_gz_yolov8_image
```

### Build Custom Image
```commandline
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git
cd PX4-ROS2-Gazebo-YOLOv8
docker build -t px4_ros2_gz_yolov8_image .
```

### Run The Docker
```commandline
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
docker run --privileged -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --network=host --ipc=host --shm-size=2gb \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --rm --name px4_ros2_gz_yolov8_container \
  px4_ros2_gz_yolov8_image
```

### What Launches in Docker
The container starts a single tmux window with 6 tiled panes:

| Pane | Service |
|------|---------|
| 1 | Micro XRCE-DDS Agent |
| 2 | PX4 SITL (x500_depth drone) |
| 3 | ROS-Gazebo camera bridge |
| 4 | YOLOv8 detection display |
| 5 | Moving car (hatchback driving in circles) |
| 6 | Keyboard drone controller |

Switch between panes with `Ctrl+b` then arrow keys.

## Keyboard Controls

All keyboard input is handled directly in the terminal (no separate window needed).

### Flight Controls
| Key | Action |
|-----|--------|
| `r` | Arm the drone |
| `l` | Land |
| `w` / `s` | Throttle up / down |
| `a` / `d` | Yaw left / right |
| Arrow keys | Roll / Pitch |
| `i` | Print flight mode |
| `Ctrl+C` | Quit |

### Gimbal Camera Controls
| Key | Action |
|-----|--------|
| `j` / `k` | Gimbal pitch down / up |
| `n` / `m` | Gimbal yaw left / right |

The camera starts at 45 degrees downward. Pitch range: -90 to +30 degrees. Yaw range: -90 to +90 degrees.

## Gimbal Camera System

The drone's camera is mounted on a 2-axis gimbal with pitch and yaw control. During Docker build, `setup_gimbal.py` modifies the x500_depth drone model SDF to replace the fixed camera joint with:

- **gimbal_yaw_joint**: Revolute joint around the Z axis (base_link to gimbal_link)
- **gimbal_pitch_joint**: Revolute joint around the Y axis (gimbal_link to camera_link)

Each joint is controlled by a `JointPositionController` plugin responding to Gazebo transport topics:
- `/gimbal/cmd_pitch` — pitch angle command
- `/gimbal/cmd_yaw` — yaw angle command

## Moving Car

`move_car.py` drives the `hatchback_blue_1` model in a circle within the simulation using `gz service /world/default/set_pose`. This provides a moving target for the YOLOv8 detection system to track.

## Manual Installation
### Create a virtual environment
```commandline
# create
python -m venv ~/px4-venv

# activate
source ~/px4-venv/bin/activate
```
### Clone repository
```commandline
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git
```
### Install PX4
```commandline
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
### Install ROS 2
```commandline
cd ~
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
pip install --user -U empy pyros-genmsg setuptools
```
### Setup Micro XRCE-DDS Agent & Client
```commandline
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
### Build ROS 2 Workspace
```commandline
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build

mkdir -p ~/ws_offboard_control/src/
cd ~/ws_offboard_control/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
```
### Install MAVSDK
> **Note:** Some users have reported installation conflicts when installing
> `ros-humble-ros-gzgarden` on a fresh setup (see issue #30 and the related
> upstream bug gazebosim/ros_gz#755). If you run into this, the recommended
> workaround is to use the prebuilt Docker image (see the Docker section above)
> until the upstream issue is resolved.

```commandline
pip install mavsdk
pip install aioconsole
sudo apt install ros-humble-ros-gzgarden
pip install numpy
pip install opencv-python
```
### Install YOLO
```commandline
pip install ultralytics
```
### Additional Configs
- Put below lines in your bashrc:
```commandline
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models
```
- Copy the content of models from main repo to ~/.gz/models
- Copy default.sdf from worlds folder in the main repo to ~/PX4-Autopilot/Tools/simulation/gz/worlds/
- Set up the gimbal camera by running `setup_gimbal.py` against your PX4 model SDF (edit `MODEL_PATH` in the script to match your PX4 installation path)

## Run
### Fly using Keyboard
You need several terminals.
```commandline
Terminal #1:
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

Terminal #2:
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4

Terminal #3:
ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera

Terminal #4:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python uav_camera_det.py

Terminal #5:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python move_car.py

Terminal #6:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python keyboard-mavsdk-test.py
```
Focus on the keyboard controller terminal, then press `r` to arm the drone. Use WASD and arrow keys for flight, `j`/`k`/`n`/`m` for gimbal control, and `l` for landing.

### Fly using ROS 2
You need several terminals.
```commandline
Terminal #1:
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

Terminal #2:
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="283.08,-136.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4

Terminal #3:
ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera

Terminal #4:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python uav_camera_det.py

Terminal #5:
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run px4_ros_com offboard_control
```

## Acknowledgement
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/
