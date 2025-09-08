#!/bin/bash

CONTAINER_NAME=isaac_ros_dev_rviz_1
IMAGE_ID=9b7fd6fc440e
WORKSPACE_DIR=/home/uwb/ws/isaac_ros-dev

xhost +local:root >/dev/null 2>&1 || true

if ! sudo docker container inspect "$CONTAINER_NAME" > /dev/null 2>&1; then
	echo "Creating new container:  $CONTAINER_NAME"
	sudo docker run -it -d \
	--runtime nvidia \
	--network host \
	--privileged \
	--volume "/tmp/.X11-unix:/tmp/.X11-unix" \
	--volume "$WORKSPACE_DIR:/ws/isaac_ros-dev" \
	--volume "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
	--volume "/dev:/dev" \
	--volume "/run/udev:/run/udev:ro" \
	--volume "/etc/udev/rules.d:/etc/udev/rules.d" \
	--env "DISPLAY=$DISPLAY" \
	--env "QT_X11_NO_MITSHM=1" \
	--name "$CONTAINER_NAME" \
	"$IMAGE_ID"
	
fi

sudo rmmod cp210x
sudo rmmod usbserial
sudo modprobe usbserial maxsize=512
sudo modprobe cp210x

sudo docker start "$CONTAINER_NAME"
sudo docker exec -it "$CONTAINER_NAME" bash
