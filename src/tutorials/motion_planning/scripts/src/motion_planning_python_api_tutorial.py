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


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    lite6 = moveit.get_planning_component("lite6")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################
    lite6.set_start_state_to_current_state()
    logger.info(f"Start State: {lite6.get_start_state().joint_positions}")

    # instantiate a RobotState instance using the current robot model
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    lite6.set_goal_state(robot_state=robot_state)
    
    # plan to goal
    plan_result = lite6.plan()

    # execute the plan
    #time.sleep(3600)
    if plan_result:
        robot_trajectory = plan_result.trajectory
        moveit.execute(robot_trajectory, controllers=[])

if __name__ == "__main__":
    main()
