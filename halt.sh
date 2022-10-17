#!/bin/bash -vx
docker stop -t 0 sim-server
docker stop -t 0 client
docker stop -t 0 ros-gui
docker stop -t 0 ros-master
docker network rm net-sim