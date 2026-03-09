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
mono_camera = False
stereo_camera = False
fts = False

def validate_args():
     #can have up to 4(no additional arguments) - 9(5 additional args) arguments
    length = len(sys.argv) - 4
    global dof
    global bhand
    global mono_camera
    global stereo_camera
    global fts

    for x in range(length):
        if (sys.argv[x+4] == "dof:=7"):
            dof = "7"
        elif (sys.argv[x+4] == "dof:=4"):
            dof = "4"
        elif ((sys.argv[x+4] == "mono_camera:=1") or (sys.argv[x+4] == "mono_camera:=true")):
            mono_camera = 1
            bhand = True
        elif ((sys.argv[x+4] == "stereo_camera:=1") or (sys.argv[x+4] == "stereo_camera:=true")):
            stereo_camera = True
            bhand = True
        elif ((sys.argv[x+4] == "bhand:=1") or (sys.argv[x+4] == "bhand:=true")):
            bhand = True
        elif ((sys.argv[x+4] == "fts:=1") or (sys.argv[x+4] == "fts:=true")):
            fts = True

def generate_launch_description():
    cmd = ['roslaunch', 'wam_sim_node', 'wam_sim_node.launch']
    validate_args()
    #Set to false if incorrect args specified. ros1_bridge is not launched 
    launch_bridge = True
    if (dof == "4") and (not fts) and (not bhand) and (not mono_camera) and (not stereo_camera):
        #4dof sim, no fts, bhand or camera    
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof:=1']) #launch sim node for ROS1
    elif (dof == "7") and (not fts) and (not bhand) and (not mono_camera) and (not stereo_camera):
        #7dof sim, no fts, bhand or camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof:=1']) #launch sim node for ROS1
    elif (dof == "4") and (fts) and (not bhand) and (not mono_camera) and (not stereo_camera):
        #4dof sim with FTS, no bhand or camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_fts:=1']) #launch sim node for ROS1
    elif (dof == "7") and (fts) and (not bhand) and (not mono_camera) and (not stereo_camera):
        #7dof sim with FTS, no bhand or camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_fts:=1']) #launch sim node for ROS1
    elif (dof == "4") and (not fts) and (bhand) and (not mono_camera) and (not stereo_camera):
        #4dof sim with Bhand, no FTS or camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_bhand:=1']) #launch sim node for ROS1
    elif (dof == "7") and (not fts) and (bhand) and (not mono_camera) and (not stereo_camera):
        #7dof sim with Bhand, no FTS or camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_bhand:=1']) #launch sim node for ROS1
    elif (dof == "4") and (fts) and (bhand) and (not mono_camera) and (not stereo_camera):
        #4dof sim with FTS and Bhand, no camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_fts_bhand:=1']) #launch sim node for ROS1
    elif (dof == "7") and (fts) and (bhand) and (not mono_camera) and (not stereo_camera):
        #7dof sim with FTS and Bhand, no camera
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_fts_bhand:=1']) #launch sim node for ROS1
    elif (dof == "4") and (not fts) and (bhand) and (mono_camera) and (not stereo_camera):
        #4dof sim with bhand and mono camera, no FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_bhand:=1', 'mono_camera:=1']) #launch sim node for ROS1
    elif (dof == "7") and (not fts) and (bhand) and (mono_camera) and (not stereo_camera):
        #7dof sim with bhand and mono camera, no FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_bhand:=1', 'mono_camera:=1']) #launch sim node for ROS1
    elif (dof == "4") and (fts) and (bhand) and (mono_camera) and (not stereo_camera):
        #4dof sim with bhand and mono camera and FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_fts_bhand:=1', 'mono_camera:=1']) #launch sim node for ROS1
    elif (dof == "7") and (fts) and (bhand) and (mono_camera) and (not stereo_camera):
        #7dof sim with bhand and mono camera and FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_fts_bhand:=1', 'mono_camera:=1']) #launch sim node for ROS1
    elif (dof == "4") and (not fts) and (bhand) and (not mono_camera) and (stereo_camera):
        #4dof sim with bhand and stereo camera, no FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_bhand:=1', 'stereo_camera:=1']) #launch sim node for ROS1
    elif (dof == "7") and (not fts) and (bhand) and (not mono_camera) and (stereo_camera):
        #7dof sim with bhand and stereo camera, no FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_bhand:=1', 'stereo_camera:=1']) #launch sim node for ROS1
    elif (dof == "4") and (fts) and (bhand) and (not mono_camera) and (stereo_camera):
        #4dof sim with bhand and stereo camera and FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '4dof_fts_bhand:=1', 'stereo_camera:=1']) #launch sim node for ROS1
    elif (dof == "7") and (fts) and (bhand) and (not mono_camera) and (stereo_camera):
        #7dof sim with bhand and stereo camera and FTS
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', '7dof_fts_bhand:=1', 'stereo_camera:=1']) #launch sim node for ROS1
    elif (dof == "0") and (not fts) and (bhand) and (not mono_camera) and (not stereo_camera):
        #standalone bhand sim 
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', 'bhand:=1']) #launch sim node for ROS1
    elif (dof == "0") and (fts) and (bhand) and (not mono_camera) and (not stereo_camera):
        #standalone bhand sim with fts
        p1 = subprocess.Popen(['roslaunch' , 'wam_sim_node', 'wam_sim_node.launch', 'bhand_fts:=1']) #launch sim node for ROS1
    else:
        launch_bridge = False
        print("Invalid configuration Specified.\nUsage: ros2 launch wam_sim wam_sim.launch.py dof:=<4 or 7> fts:=<true/false> bhand:=<true/false> mono_camera:=<true/false> stereo_camera:=<true/false>")
    if (launch_bridge):
        return LaunchDescription([Node(package="ros1_bridge", node_executable="dynamic_bridge", output="screen", arguments=["--bridge-all-topics", "__log_disable_rosout:=true"]),]) #launch ROS1 bridge for all topics except rosout (for fastRTPS).

