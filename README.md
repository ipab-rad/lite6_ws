# ufactory_lite6_ws
A default workspace configuration for the ufactory lite6 robot.
<img src="./assets/workspace.jpg" width="400" />

# Actions Monitor 🔎
| **Workflow Name**            | **Description**                                        | **Status**                                                                                                                                                                                                                                                                      |
|------------------------------|--------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| control    | Publishes a built controller image for lite6 ROS2 workspace.       | [![lite6_control](https://github.com/ipab-rad/lite6_ws/actions/workflows/control.yaml/badge.svg)](https://github.com/ipab-rad/lite_6/blob/rolling/.github/workflows/control.yaml)          |
| motion_planning    | Publishes a built motion_planning image for lite6 ROS2 workspace.       | [![motion_planning](https://github.com/ipab-rad/lite6_ws/actions/workflows/motion_planning.yaml/badge.svg)](https://github.com/ipab-rad/lite_6/blob/rolling/.github/workflows/motion_planning.yaml)          |

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
