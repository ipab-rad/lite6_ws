#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints

from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    
    # read moveit config files
    moveit_config = (
        MoveItConfigsBuilder(robot_name="lite6", package_name="moveit_resources_lite6_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic("config/lite6.srdf")
        .robot_description(file_path=get_package_share_directory("moveit_resources_lite6_description") 
            + "/urdf/lite6.urdf")
        .moveit_cpp(
            file_path=get_package_share_directory("lite6_motion_planning_demos")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    ).to_dict()

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    lite6 = moveit.get_planning_component("lite6")
    logger.info("MoveItPy instance created")


    # Plan 1 - demonstrate moving robot to random joint positions
    
    lite6.set_start_state_to_current_state()
    logger.info(f"Start State: {lite6.get_start_state().joint_positions}")

    # instantiate a RobotState instance using the current robot model
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set goal state to the initialized robot state
    logger.info("Set goal state to random valid robot state")
    lite6.set_goal_state(robot_state=robot_state)
    
    # plan to goal
    plan_result = lite6.plan()

    # execute the plan
    if plan_result:
        robot_trajectory = plan_result.trajectory
        input("Press Enter to execute the trajectory")
        moveit.execute(robot_trajectory, controllers=[])
    
    # Plan 2 - reset robot to home position
    
    lite6.set_start_state_to_current_state()

    # set goal state to the home position
    logger.info("Set goal state to the home position")
    lite6.set_goal_state(configuration_name="home")
    
    # plan to goal
    plan_result = lite6.plan()

    # execute the plan
    if plan_result:
        robot_trajectory = plan_result.trajectory
        input("Press Enter to execute the trajectory")
        moveit.execute(robot_trajectory, controllers=[])

if __name__ == "__main__":
    main()
