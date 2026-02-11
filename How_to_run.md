# How to Run

Two autopilot stacks are supported. Choose based on your hardware:

- **ArduPilot** (default) — for Pixhawk 6X/6C and other ArduCopter-based hardware
- **PX4** — for PX4-based flight controllers

## Option 1: ArduPilot (Recommended)

### Quick Start
```bash
bash run_ardupilot.sh
```

### Manual Steps
```bash
# Stop any existing container
docker rm -f ardupilot_yolov8_container 2>/dev/null

# Build
docker build -t ardupilot_yolov8_image .

# Allow X11 forwarding
xhost +local:docker

# Run
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
  --rm --name ardupilot_yolov8_container \
  ardupilot_yolov8_image
```

## Option 2: PX4

```bash
# Stop any existing container
docker rm -f px4_ros2_gazebo_yolov8_container 2>/dev/null

# Build using the PX4 Dockerfile
docker build -f Dockerfile.px4 -t px4_ros2_gazebo_yolov8_image .

# Allow X11 forwarding
xhost +local:docker

# Run
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
  --rm --name px4_ros2_gazebo_yolov8_container \
  px4_ros2_gazebo_yolov8_image
```
