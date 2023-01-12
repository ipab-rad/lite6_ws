ARG ROS_DISTRO=rolling
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
    apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --mixin debug

# build the workspace
#RUN sudo chmod 777 -R . && colcon build --mixin debug
