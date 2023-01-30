# ufactory_lite6_ws
A default workspace configuration for the ufactory lite6 robot.

# Actions Monitor 🔎
| **Workflow Name**            | **Description**                                        | **Status**                                                                                                                                                                                                                                                                      |
|------------------------------|--------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| deploy-base-image    | Publishes a built base image for lite6 ROS2 workspace.       | [![deploy-base-image](https://github.com/ipab-rad/lite6_ws/actions/workflows/deploy_base_image.yaml/badge.svg)](https://github.com/ipab-rad/lite_6/blob/humble/.github/workflows/deploy_base_image.yaml)          |
| deploy-perception-image    | Publishes a built perception image for lite6 ROS2 workspace.       | [![deploy-perception-image](https://github.com/ipab-rad/lite6_ws/actions/workflows/deploy_perception_image.yaml/badge.svg)](https://github.com/ipab-rad/lite_6/blob/humble/.github/workflows/deploy_perception_image.yaml)          |

# Instructions
If you wish to run a container as a GUI application you need to first manage X-server authentication. The most basic way to do so is through enabling access to all local applications by running: 

```
xhost +local:
```

This is in general bad practice as it disables security settings for local applications. In order to authenticate the docker container alone the following command needs to be run to populate a temporary file which our docker build will use:

```
export DOCKER_XAUTH=/tmp/.docker.xauth
touch $DOCKER_XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $DOCKER_XAUTH nmerge -
```

In order to build the container one must then run: 

```
docker-compose -f <docker-compose-file> build
```

To run the application one must run with the correct IP address for the panda robot:

```
docker-compose -f <docker-compose-file> up
```

# Realsense System Dependencies
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

# Zedd System Dependencies
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
