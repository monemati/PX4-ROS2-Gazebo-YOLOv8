#!/bin/bash
# Build and run the PX4 simulation in Docker
set -e

docker rm -f px4_ros2_gazebo_yolov8_container 2>/dev/null || true

docker build -f Dockerfile.px4 -t px4_ros2_gazebo_yolov8_image .

xhost +local:docker 2>/dev/null

XAUTH=/tmp/.docker.xauth
touch "$XAUTH"
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -

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
