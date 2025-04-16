#Packages to get address of the YAML file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    movement_node = Node(name="movement",
                       package='act1_2',
                       executable='movement',
                       emulate_tty=True,
                       output='screen',
                       )
    vision_node = Node(name="vision",
                       package='act1_2',
                       executable='vision',
                       emulate_tty=True,
                       output='screen',
                       )
    
    ball_node = Node(name="ball",
                     package='act1_2',
                     executable='ball',
                     emulate_tty=True,
                     output='screen',
                    )
    
    pid_follower_node = Node(name="pid_follower",
                     package='act1_2',
                     executable='pid_follower',
                     emulate_tty=True,
                     output='screen',
                    )
    
    l_d = LaunchDescription([movement_node, vision_node, ball_node, pid_follower_node])

    return l_d