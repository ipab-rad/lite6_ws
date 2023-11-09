#!/usr/bin/env python3
"""
Shows how to use a planning scene in MoveItPy to add collision objects and perform collision checking.
"""

import time
import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        input("Press Enter to execute trajectory")
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def add_collision_objects(planning_scene_monitor):
    """Helper function that adds collision objects to the planning scene."""
    object_positions = [
        (0.15, 0.1, 0.5),
        (0.25, 0.0, 1.0),
        (-0.25, -0.3, 0.8),
        (0.25, 0.3, 0.75),
    ]
    object_dimensions = [
        (0.1, 0.4, 0.1),
        (0.1, 0.4, 0.1),
        (0.2, 0.2, 0.2),
        (0.15, 0.15, 0.15),
    ]

    with planning_scene_monitor.read_write() as scene:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "link_base"
        collision_object.id = "boxes"

        for position, dimensions in zip(object_positions, object_dimensions):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions

            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()  # Important to ensure the scene is updated


def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py_planning_scene")

    # instantiate MoveItPy instance and get planning component
    lite6 = MoveItPy(node_name="moveit_py_planning_scene")
    lite6_arm = lite6.get_planning_component("lite6")
    planning_scene_monitor = lite6.get_planning_scene_monitor()
    logger.info("MoveItPy instance created")

    ###################################################################
    # Plan with collision objects
    ###################################################################

    add_collision_objects(planning_scene_monitor)
    lite6_arm.set_start_state_to_current_state()
    lite6_arm.set_goal_state(configuration_name="home")
    plan_and_execute(lite6, lite6_arm, logger, sleep_time=3.0)

    ###################################################################
    # Check collisions
    ###################################################################
    with planning_scene_monitor.read_only() as scene:
        robot_state = scene.current_state
        original_joint_positions = robot_state.get_joint_group_positions("lite6")

        # Set the pose goal
        pose_goal = Pose()
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.25
        pose_goal.position.z = 0.5
        pose_goal.orientation.w = 1.0

        # Set the robot state and check collisions
        robot_state.set_from_ik("lite6", pose_goal, "link6")
        robot_state.update()  # required to update transforms
        robot_collision_status = scene.is_state_colliding(
            robot_state=robot_state, joint_model_group_name="lite6", verbose=True
        )
        logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

        # Restore th original state
        robot_state.set_joint_group_positions(
            "lite6",
            original_joint_positions,
        )
        robot_state.update()  # required to update transforms

    time.sleep(3.0)

    ###################################################################
    # Remove collision objects and return to the ready pose
    ###################################################################

    with planning_scene_monitor.read_write() as scene:
        scene.remove_all_collision_objects()
        scene.current_state.update()

    lite6_arm.set_start_state_to_current_state()
    lite6_arm.set_goal_state(configuration_name="ready")
    plan_and_execute(lite6, lite6_arm, logger, sleep_time=3.0)


if __name__ == "__main__":
    main()
