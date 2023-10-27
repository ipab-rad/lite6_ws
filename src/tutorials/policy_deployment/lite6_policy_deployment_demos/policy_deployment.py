#!/usr/bin/env python3
"""Demonstrating policy deployment on lite6."""

import jax
import flax

import rclpy

from moveit.policies import Policy

from geometry_msgs.msg import PoseStamped

from lite6_policy_deployment_demos.lite6_policy_deployment_demos_parameters import policy as params

class DummyPolicy(Policy):
    """Dummy policy for testing."""

    def __init__(self):
        super().__init__(params)
        self._logger.info("Dummy policy initialized")

    def forward(self, sensor_msg):
        """Forward pass of the policy."""
        if self.active:
            # add code to perform forward pass through your neural network here

            # convert neural network output to a pose message
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = "base_link"

            # publish target pose
            self.controller_pub.publish(target)


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("policy_deployment")
    policy = DummyPolicy()
    rclpy.spin(policy)
    policy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
