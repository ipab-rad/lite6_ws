from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node


def generate_launch_description():
    
    policy_params = {
            "policy": ParameterBuilder("lite6_policy_deployment_demos")
            .yaml("config/policy.yaml")
            .to_dict()
            }

    policy_node = Node(
            package="lite6_policy_deployment_demos",
            executable="policy_deployment",
            parameters=[policy_params],
            output="both",
            )

    return LaunchDescription([policy_node])
