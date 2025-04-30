#Packages to get address of the YAML file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    movement_node = Node(name="robot_movement_node",
                       package='act1_4',
                       executable='robot_movement',
                       emulate_tty=True,
                       output='screen',
                       )
                       
    bridge_node = Node(name="teleop_bridge_node",
                       package='act1_4',
                       executable='teleop_bridge',
                       emulate_tty=True,
                       output='screen',
                       )

    l_d = LaunchDescription([movement_node, bridge_node])

    return l_d
