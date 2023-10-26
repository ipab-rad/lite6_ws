#!/usr/bin/env python3
"""Demonstrating policy deployment on lite6."""

import rclpy

from moveit.policies import Policy

if __name__ == "__main__":
    rclpy.init()
    policy = Policy()
    
    import time
    time.sleep(1000)
