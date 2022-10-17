#!/bin/bash -xve
X=${1:-2.05}
Y=${2:-2.68}
docker exec -it client /opt/ep_ws/devel/env.sh \
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: 
  seq: 0 
  stamp:  
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: $X
    y: $Y
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0  
    w: 1.0"
