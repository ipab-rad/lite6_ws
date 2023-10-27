[![lite6_control](https://github.com/ipab-rad/lite6_ws/actions/workflows/control.yaml/badge.svg)](https://github.com/ipab-rad/lite6_ws/blob/rolling/.github/workflows/control.yaml) [![motion_planning](https://github.com/ipab-rad/lite6_ws/actions/workflows/motion_planning.yaml/badge.svg)](https://github.com/ipab-rad/lite6_ws/blob/rolling/.github/workflows/motion_planning.yaml) [![zed](https://github.com/ipab-rad/lite6_ws/actions/workflows/zed.yaml/badge.svg)](https://github.com/ipab-rad/lite6_ws/blob/rolling/.github/workflows/zed.yaml)
# UFactory Lite6 Workspace
This workspace serves as a template for setting up a ROS 2 workspace for performing robot learning research. 

<img src="./assets/workspace.jpg" width="400" />

# Hardware Setup
<img src="./assets/ufactory.png" width="400" />

# Software Setup

## System Software Prerequisites
This workspaces requires the following software to be installed:

* An installation of Docker ([instructions](https://docs.docker.com/engine/install/ubuntu/))
* An installation of ROS rolling ([instructions](https://docs.ros.org/en/rolling/Installation.html))

## Build ROS workspace on host machine
In order to install and build all workspace dependencies on your local machine all you need to run is:

```bash
./local_setup.sh
```

This is only necessary if you wish to run application code directly on the host machine, most applications within this repository are dockerised and hence don't require you to build the ROS 2 workspace on the host machine.

## Docker GUI Prerequisites
If you wish to run a container that contains GUI applications (e.g. rviz) you need to first manage X-server authentication. The most basic way to do so is through enabling access to all local applications by running: 

```
xhost +local:
```

This is in general bad practice as it disables security settings for local applications. In order to authenticate the docker container alone the following command needs to be run to populate a temporary file which our docker build will use:

```
export DOCKER_XAUTH=/tmp/.docker.xauth
touch $DOCKER_XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $DOCKER_XAUTH nmerge -
```

## Running Tutorials

### Motion Planning Tutorial
```bash
ros2 launch 
```
Please see [motion_planning_tutorial](placeholder.com) for further instructions.


### Control Tutorial

Start the application code (control server, servo node, foxglove visualization):
```bash
cd .docker/control
docker compose -f docker-compose-servo-application.yaml up
```

Note: the current set of configs leverage node constraints to deploy containers on specific devices in a Docker swarm, more details about this config will be provided as the repository moves from development to stable state.

Run example trajectory code: 
```bash
source /opt/ros/rolling/setup.bash
python ./src/tutorials/control/src/servo.py 
```

Note: in its current state, for visualization code to work one needs to run the foxglove bridge with 

```
ros2 run foxglove_bridge foxglove_bridge
```

Please see [control_tutorial](placeholder.com) for further instructions.

### Policy Deployment Tutorial
```bash
ros2 launch 
```
Please see [policy_deployment_tutorial](placeholder.com) for further instructions.
