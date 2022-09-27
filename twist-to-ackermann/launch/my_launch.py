import os
import pathlib
import launch_ros.actions
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    TwistPublisherNode = Node(
        package = "twist-to-ackermann",
        executable = "TwistPublisher",
        name = "twist_publisher_node"
    )
    
    ConversionNode = Node(
        package = "twist-to-ackermann",
        executable = "Conversion",
        name = "conversion_node"
    )
    
    return LaunchDescription([
        TwistPublisherNode,
        ConversionNode
    ])