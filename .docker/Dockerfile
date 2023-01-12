ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
MAINTAINER Peter David Fagan "peterdavidfagan@gmail.com"

# set default DDS to Cyclone
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# set up workspace
WORKDIR /root/panda_ws
COPY . .

# import dependent libraries
RUN vcs import --skip-existing src < src/moveit2/moveit2.repos && \
    vcs import --skip-existing src < src/moveit2_tutorials/moveit2_tutorials.repos 

# install ros dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get -q update && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# build the workspace
RUN colcon build --mixin debug
