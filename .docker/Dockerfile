ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
MAINTAINER Peter David Fagan "peterdavidfagan@gmail.com"

# set default DDS to Cyclone
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# set up workspace
ENV ROS_UNDERLAY /root/lite6_ws/install
WORKDIR $ROS_UNDERLAY/..
COPY . .

# install ros dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-moveit \
    wget && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    vcs import --skip-existing src < src/moveit2_tutorials/moveit2_tutorials.repos && \ 
    rosdep update && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    colcon build --mixin debug && \
    rm -rf /var/lib/apt/lists/* && \
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.bash#g" /ros_entrypoint.sh
