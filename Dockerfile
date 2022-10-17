FROM ros:noetic-ros-core-focal as ROS

##########################################################################
FROM docker.discover-lab.com:55555/rmus-2022-fall/client-cpu

COPY --from=ROS /etc/apt /etc/apt
RUN sed -i "s@http://.*archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-noetic-map-server ros-noetic-rqt-reconfigure ros-noetic-iris-lama-ros ros-noetic-global-planner \
    cmake libeigen3-dev libcppunit-dev python3-psutil python3-future && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple PyQt5
# RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple evo --upgrade --no-binary evo
# ADD PyKDL.so /opt/conda/lib/python3.7/site-packages/PyKDL.so

ADD orocos_kinematics_dynamics /opt/orocos_kinematics_dynamics
RUN mkdir /opt/orocos_kinematics_dynamics/orocos_kdl/build
WORKDIR /opt/orocos_kinematics_dynamics/orocos_kdl/build
RUN CMAKE_INSTALL_PREFIX=/opt/conda cmake .. && make && make install

WORKDIR /opt/orocos_kinematics_dynamics/python_orocos_kdl/build
RUN CMAKE_INSTALL_PREFIX=/opt/conda ROS_PYTHON_VERSION=3 cmake .. && make
RUN cp PyKDL.so /opt/conda/lib/python3.7/site-packages/PyKDL.so

WORKDIR /opt
# RUN rm -rf /opt/ep_ws/src/sim2real_ep/carto_navigation
ARG CACHE_DATE="tmp"
ADD navigation /opt/ep_ws/src/sim2real_ep/navigation
WORKDIR /opt/ep_ws
RUN rm build/CMakeCache.txt && /opt/workspace/devel_isolated/env.sh catkin_make