#!/bin/bash
# Build and run the ArduPilot simulation in Docker
set -e

docker rm -f ardupilot_yolov8_container 2>/dev/null || true

docker build -t ardupilot_yolov8_image .

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
  --rm --name ardupilot_yolov8_container \
  ardupilot_yolov8_image
