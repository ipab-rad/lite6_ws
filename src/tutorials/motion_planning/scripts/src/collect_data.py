#!/usr/bin/env python3
"""A basic data collection script."""
import os

import rclpy
from rclpy.node import Node

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

from moveit_msgs.srv import ServoCommandType
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import numpy as np

class DataCollector(Node):
    """A basic data collection node."""

    def __init__(self):
        super().__init__("data_collector")
        self.logger = self.get_logger()
        
        # waypoints
        t = np.linspace(0, 20, 100)
        x = ((np.sin(t) + 1) / 10) + 0.1
        y = ((np.cos(t) + 1) / 10)
        z = (t / 50) + 0.1

        # generate list of poses
        self.poses = []
        for i in range(len(t)):
            pose = PoseStamped()
            pose.header.frame_id = "link_base"
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            pose.pose.position.z = z[i]
            pose.pose.orientation.x = 0.924
            pose.pose.orientation.y = -0.382
            pose.pose.orientation.z = 0.0   
            pose.pose.orientation.w = 0.0
            self.poses.append(pose)
       
        print(self.poses)
        self.target_pose = self.set_target_pose(self.poses[0])
        # initialize motion planning client
        moveit_config = (
        MoveItConfigsBuilder(robot_name="lite6", package_name="moveit_resources_lite6_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic("config/lite6.srdf")
        .robot_description(file_path=get_package_share_directory("moveit_resources_lite6_description") 
            + "/urdf/lite6.urdf")
        .moveit_cpp(
            file_path=get_package_share_directory("lite6_moveit_demos")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
        ).to_dict()

        self.robot = MoveItPy(config_dict=moveit_config)
        self.arm = self.robot.get_planning_component("lite6")

        # initialize servo service client
        self.servo_command_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        while not self.servo_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.servo_pause_client = self.create_client(SetBool, "/servo_node/pause_servo")
        while not self.servo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # initialize servo target pose publisher
        self.publisher = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)
        
        # initialize timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_target_pose)
        
        
    def publish_target_pose(self):
        """Publishes the target pose."""
        # check if within threshold of target pose
        self.set_target_pose(self.poses[0])
        current_pose = self.get_current_pose()
        del_x = current_pose.position.x - self.target_pose.pose.position.x
        del_y = current_pose.position.y - self.target_pose.pose.position.y
        del_z = current_pose.position.z - self.target_pose.pose.position.z
        print(np.linalg.norm([del_x, del_y, del_z]))
        if np.linalg.norm([del_x, del_y, del_z]) < 0.05:
            self.get_logger().info("Reached target pose: %s" % self.target_pose)
            # pop pose from list
            self.poses.pop(0)
            # check if list is empty
            if len(self.poses) == 0:
                self.get_logger().info("Finished collecting data.")
                self.return_to_start()
                self.timer.cancel()
                return

            # set new target pose
            self.logger.info("Target pose set to: %s" % self.poses[0])
            self.set_target_pose(self.poses[0])
            
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.target_pose)
    
    def set_servo_command_type(self, command_type):
        """Sets the servo command type."""
        request = ServoCommandType.Request()
        request.command_type = command_type
        self.future = self.servo_command_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result():
            self.get_logger().info("Servo command type set to: %s" % command_type)
        else:
            self.get_logger().error("Failed to set servo command type: %s" % command_type)
    
    def get_current_pose(self):
        """Gets the current pose of the robot."""
        self.arm.set_start_state_to_current_state()
        robot_state = self.arm.get_start_state()
        pose = robot_state.get_pose("link_eef")
        return pose
    
    def return_to_start(self):
        """Returns the robot to the start position."""
        # pause servo    
        request = SetBool.Request()
        request.data = True
        future = self.servo_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.logger.info("servo paused")

        # plan to ready config
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")
        plan = self.arm.plan()
        if plan:
            robot_trajectory = plan.trajectory
            self.robot.execute(robot_trajectory, controllers=[])

        # start servo
        request = SetBool.Request()
        request.data = False
        future = self.servo_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.logger.info("servo started")
    
    def set_target_pose(self, pose):
        """Sets the target pose."""
        self.target_pose = pose

    def collect_data(self):
        """Collects data by moving the robot through poses."""
        # generate some waypoints
        t = np.linspace(0, 20, 20)
        x = ((np.sin(t) + 1) / 6) + 0.3
        y = ((np.cos(t) + 1 ) / 6)
        z = (t / 30) + 0.3

        print("x: ", x)
        print("y: ", y)

        # move to each waypoint
        for i in range(len(t)):
            self.arm.set_start_state_to_current_state()
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "link_base"
            pose_goal.header.stamp = self.get_clock().now().to_msg()
            pose_goal.pose.orientation.x = 0.924
            pose_goal.pose.orientation.y = -0.382
            pose_goal.pose.orientation.z = 0.0
            pose_goal.pose.orientation.w = 0.0
            pose_goal.pose.position.x = x[i]
            pose_goal.pose.position.y = y[i]
            pose_goal.pose.position.z = z[i]
            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_eef")

            plan = self.arm.plan()
            if plan:
                robot_trajectory = plan.trajectory
                self.robot.execute(robot_trajectory, controllers=[])
    
    def collect_data_1(self):
        """Collects data by moving the robot through poses by setting new target poses."""
        # generate some waypoints
        t = np.linspace(0, 20, 20)
        x = ((np.sin(t) + 1) / 6) + 0.3
        y = ((np.cos(t) + 1 ) / 6)
        z = (t / 30) + 0.3

        # set target based on proximity to waypoint
        for i in range(len(t)):
            # set target pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "link_base"
            pose.pose.orientation.x = 0.924
            pose.pose.orientation.y = -0.382
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            pose.pose.position.z = z[i]
            self.set_target_pose(pose)
            while True:
                # get current pose
                pose = self.get_current_pose()
                # check if close enough to waypoint
                if np.linalg.norm(np.array([pose.position.x, pose.position.y, pose.position.z]) - np.array([x[i], y[i], z[i]])) < 0.01:
                    break
                self.publisher.publish(self.target_pose)


    def pick(self):
        """Basic object picking command."""
        pass

    def place(self):
        """Basic object placing command."""
        pass



if __name__=="__main__":
    rclpy.init()
    node = DataCollector()
    
    # return to start position
    node.return_to_start()

    # set target pose
#    pose = PoseStamped()
#    pose.header.frame_id = "panda_link0"
#    pose.pose.position.x = 0.307
#    pose.pose.position.y = 0.0
#    pose.pose.position.z = 0.59 + 0.2
#    pose.pose.orientation.x = 0.924
#    pose.pose.orientation.y = -0.382
#    pose.pose.orientation.z = 0.0
#    pose.pose.orientation.w = 0.0
#    node.set_target_pose(pose)

    # set servo command type
    node.set_servo_command_type(2)

    # TODO: for demo include service call
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
