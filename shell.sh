#!/bin/bash -xve
EXE=${@:-bash}
docker exec -it client /opt/ep_ws/devel/env.sh $EXE
