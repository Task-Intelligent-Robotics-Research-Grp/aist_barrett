from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import os
os.system("killall roscore")
with open(os.devnull, 'w') as fp:
	p2 = subprocess.Popen(['roscore'], stdout=fp, stderr=subprocess.STDOUT) #launch a new ROS1 core, needed for bridging. Disable stdout and stderrs

p1 = subprocess.Popen(['ros2' , 'run', 'wam_node', 'wam_node']) #launch as a subprocess, so libbarrett prompts can be shown on screen
def generate_launch_description(): 
    return LaunchDescription([
        #Node(package="wam_node", node_executable="wam_node", output="screen"),
	Node(package="ros1_bridge", node_executable="dynamic_bridge", arguments=["--bridge-all-topics", "__log_disable_rosout:=true"], output="log"), 
        ])



