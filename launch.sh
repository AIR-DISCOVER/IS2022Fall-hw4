#!/bin/bash -evx
xhost +
ID=test
docker build . -t docker.discover-lab.com:55555/$ID/client:hw4 --build-arg CACHE_DATE="$(date +%Y-%m-%d:%H:%M:%S)" --network host

# Network and core
docker network create net-sim
docker run -dit --rm --name ros-master --network net-sim \
    ros:noetic-ros-core-focal roscore

# Server
docker pull docker.discover-lab.com:55555/rmus-2022-fall/sim-headless:v4.1.0-hw4
docker run -dit --rm --name sim-server --network net-sim \
    -e ROS_MASTER_URI="http://ros-master:11311" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix 
    --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
    docker.discover-lab.com:55555/rmus-2022-fall/sim-headless:v4.1.0-hw4

# Visualization
docker run -dit --rm --name ros-gui --network net-sim \
    -e ROS_MASTER_URI=http://ros-master:11311 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    docker.discover-lab.com:55555/rmus-2022-fall/ros-gui \
    /opt/ros/noetic/env.sh rosrun image_view image_view image:=/third_rgb

# Start cli
docker run -dit --rm --network net-sim --name client \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ENV_ROBOT_MODE=sim \
    -e ROS_MASTER_URI=http://ros-master:11311 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    docker.discover-lab.com:55555/$ID/client:hw4 bash

# Start localization
docker exec -it client \
    /opt/ros/noetic/env.sh /opt/ep_ws/devel/env.sh \
    roslaunch navigation navigation.launch

./halt.sh