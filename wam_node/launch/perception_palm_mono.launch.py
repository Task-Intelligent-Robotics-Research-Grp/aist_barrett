from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import os
import sys
os.system("killall roslaunch")
p1 = subprocess.Popen(['roslaunch' , 'perception_palm', 'perception_palm.launch'])
def generate_launch_description(): 
    return LaunchDescription([
        #Node(package="wam_node", node_executable="wam_node", output="screen"),
	Node(package="ros1_bridge", node_executable="dynamic_bridge", arguments=["--bridge-all-topics", "__log_disable_rosout:=true"], output="log"), 
        ])



