import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import subprocess
import sys

dof = "0"
bhand = False
fts = False

def validate_args():
    #can have up to 4(no additional arguments) - 9(5 additional args) arguments
    length = len(sys.argv) - 4
    global dof
    global bhand
    global fts
    for x in range(length):
        if (sys.argv[x+4] == "dof:=7"):
            dof = "7"
        elif (sys.argv[x+4] == "dof:=4"):
            dof = "4"
        elif ((sys.argv[x+4] == "bhand:=1") or (sys.argv[x+4] == "bhand:=true")):
            bhand = True
        elif ((sys.argv[x+4] == "fts:=1") or (sys.argv[x+4] == "fts:=true")):
            fts = True

def generate_launch_description():
    validate_args()
    #Set to false if incorrect args specified. ros1_bridge is not launched 
    launch_rviz = True
    if (dof == "4") and (not fts) and (not bhand):
        #4dof sim, no fts, bhand or camera    
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof', 'wam4dof.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "7") and (not fts) and (not bhand):
        #7dof sim, no fts, bhand or camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof', 'wam7dof.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "4") and (fts) and (not bhand):
        #4dof sim with FTS, no bhand or camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof', 'wam4dof_fts.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "7") and (fts) and (not bhand):
        #7dof sim with FTS, no bhand or camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof', 'wam7dof_fts.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "4") and (not fts) and (bhand):
        #4dof sim with Bhand, no FTS or camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof', 'wam4dof_hand.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "7") and (not fts) and (bhand):
        #7dof sim with Bhand, no FTS or camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof', 'wam7dof_hand.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'launch','wam_config.rviz')
    elif (dof == "4") and (fts) and (bhand):
        #4dof sim with FTS and Bhand, no camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam4dof', 'wam4dof_hand_fts.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "7") and (fts) and (bhand):
        #7dof sim with FTS and Bhand, no camera
        urdf = os.path.join(get_package_share_directory('wam_sim'),'launch','wam7dof', 'wam7dof_hand_fts.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','wam_config.rviz')
    elif (dof == "0") and (bhand):
        #7dof sim with FTS and Bhand, no camera
        urdf = os.path.join(get_package_share_directory('wam_sim'), 'launch','wam7dof', 'barrett_hand.urdf')
        config = os.path.join(get_package_share_directory('wam_sim'),'config','bhand_config.rviz')
    else:
        launch_rviz = False
        print("Invalid configuration Specified.\nUsage: ros2 launch wam_sim wam_empty_sim.launch.py dof:=<4 or 7> fts:=<true/false> bhand:=<true/false>")
    if (launch_rviz):
        return LaunchDescription([Node(package="robot_state_publisher", node_executable="robot_state_publisher", output="screen", arguments=[urdf]), Node(package="rviz2", node_executable="rviz2", output="screen", arguments=["-d " + config]),])

