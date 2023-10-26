#!/usr/bin/env python3
"""Demonstrating policy deployment on lite6."""

import jax
import flax

import rclpy

from moveit.policies import Policy

from geometry_msgs.msg import PoseStamped

class DummyPolicy(Policy):
    """Dummy policy for testing."""

    def __init__(self):
        super().__init__()
        self._logger.info("Dummy policy initialized")

    def forward(self, sensor_msg):
        """Forward pass of the policy."""
        self._logger.info("Forward pass")
        self._logger.info(sensor_msg)
        return PoseStamped()

def main():
    rclpy.init()
    policy = DummyPolicy()
    rclpy.spin(policy)
    policy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
