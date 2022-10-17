#!/bin/bash -xve
EXE=${1:-bash}
docker exec -it client /opt/ep_ws/devel/env.sh $EXE