#!/bin/bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
CONTAINER_NAME=people-detection
LAUNCH_ARG=""

if [ $# -eq 0 ]; then
  echo "The script needs an argument"
  exit 1
fi

case "$1" in
  smart-assembly) LAUNCH_ARG="type:=smart_assembly" ;;
  automatic-assembly) LAUNCH_ARG="type:=automatic_assembly" ;;
  *) echo "Unsupported argument"; exit 1;;
esac

# docker build -t s3c-people-detection "${ROOT_DIR}" || exit 1

if [ "$(docker ps -aq -f status=running -f name=${CONTAINER_NAME})" ]; then
  docker stop ${CONTAINER_NAME}
  if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
    docker rm ${CONTAINER_NAME}
  fi
fi

ptpcam --show-property=0x5013 --val=0x8005

xhost +
docker run -it --rm \
  --privileged \
  --net=host \
  -e DISPLAY="${DISPLAY}" \
  -e XAUTHORITY="${XAUTHORITY}" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/dri:/dev/dri \
  --device=/dev/bus/usb:/dev/bus/usb \
  --ipc=host \
  --name ${CONTAINER_NAME} \
  s3c-people-detection roslaunch rslidar_sdk start.launch "${LAUNCH_ARG}"
