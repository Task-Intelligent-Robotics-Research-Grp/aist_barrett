# barrett-ros2-pkg
> ROS2 package for the Barrett WAM and related products

**Current CI Status**: [![Build Status](https://git.barrett.com/software/barrett-ros2-pkg/badges/devel/build.svg)](https://git.barrett.com/software/barrett-ros2-pkg/pipelines)

<!-- TOC depthFrom:1 depthTo:4 withLinks:1 updateOnSave:1 orderedList:0 -->

- [barrett-ros2-pkg](#barrett-ros2-pkg)
	- [Overview](#overview)
	- [Pre-Requisites](#pre-requisites)
	- [Installation](#installation)
	- [Networking](#networking)
	- [Running WAM Simulation](#running-wam-simulation)
	- [Running standalone BarrettHand Simulation](#running-standalone-barretthand-simulation)
	- [Running WAM Node](#running-wam-node)
	- [Running Perception Palm](#running-perception-palm)
	    - [Set up Cameras](#set-up-cameras)
	    - [Calibration](#calibration)
	    - [Running the demo](#running-the-demo)
	    - [Accessing the sensors](#accessing-the-sensors)
	    - [Troubleshooting](#troubleshooting)
	- [Running WAM Demos](#running-wam-demos)
		- [Teach and Play](#teach-and-play)
	- [Subscribed Topics](#subscribed-topics)
	    - [WAM Subscribed Topics](#wam-subscribed-topics)
	- [Published Topics](#published-topics)
	    - [WAM Published Topics](#wam-published-topics)
	    - [FTS Published Topics](#fts-published-topics)
        - [BarrettHand Published Topics](#barretthand-published-topics)
        - [Perception Palm Published Topics](#perception-palm-published-topics)
	- [Services](#services)
		- [WAM Services](#wam-services)
		- [FTS Services](#fts-services)
		- [BarrettHand Services](#barretthand-services)
		- [Perception Palm Services](#perception-palm-services)
    - [Directory Structure](#directory-structure)
	- [Contributing](#contributing)
	- [Resources](#resources)

<!-- /TOC -->

## Overview

This repository includes the following components:
- `wam_sim` - a WAM simulation. 
- `wam_node` - A ROS2 wrapper for Libbarrett used to control the WAM, BarrettHand and Force/Torque Sensor hardware
- `perception_palm` - A ROS1 driver for Barrett's Perception Palm. This library is a ROS wrapper written above the open source C/C++ library for Microchip's USB-to-SPI protocol coverter.
- `wam_demos` - A demos package that works with both `wam_node` and `wam_sim`. Currently contains a teach and play demo using rosbags (ROS1 `wam_deoms`) or CSV files (ROS2 `wam_demos`) to record and play back trajectories.
- `bhand_msgs`, `bhand_srvs`, `wam_msgs`, `wam_srvs` - Packages that contain message and service files to interface with the WAM and BarrettHand.

**Note:** The WAM, Force/Torque Sensor and BarrettHand Hardware are controlled natively using ROS2, but a ROS2-ROS1 bridge allows control with ROS1. The WAM simulation and Perception Palm hardware are natively controlled using ROS1, but a ROS1-ROS2 bridge allows control with ROS2.  

## Pre-Requisites
Start with a clean Ubuntu 18.04 installation. Create a USB installer, assumes USB stick > 2GB at /dev/sdc (verify this!):
```sh
sudo bash
apt install pv
wget http://releases.ubuntu.com/bionic/ubuntu-18.04.3-desktop-amd64.iso
dd if=ubuntu-18.04.3-desktop-amd64.iso |pv -s 2G |dd of=/dev/sdc bs=8M
```
Boot from the USB stick and install Ubuntu to the hard disk.

Perform an install of ROS 2 Dashing Diademata [via Debian](https://index.ros.org//doc/ros2/Installation/Dashing/Linux-Install-Debians/) (recommended) or [source](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/). You will need the `robot_state_publisher` package to run the wam simulator (in case you installed ROS_Base instead of ROS_Desktop):
```sh
sudo apt-get install ros-dashing-robot-state-publisher
```
Install libudev and wstool:
```sh
sudo apt-get update
sudo apt-get install libudev-dev
sudo apt-get install python-wstool
```
Install the camera driver (needed for the `perception_palm` package):
```sh
sudo apt-get install ros-melodic-camera-umd
```
You will also need the `ros1_bridge` package. In order to use custom `msg` and `srv` files with the ROS1 bridge, it needs to be built from source. Clone the git repository in a new workspace (it will be built after building the custom `msg` and `srv` files) and install it's dependencies.
```sh
sudo apt-get install git ros-dashing-launch*
cd ~
mkdir -p ros1-bridge-ws
cd ~/ros1-bridge-ws
git clone https://github.com/ros2/ros1_bridge.git
```
**Do not source or add the ROS distro to `bashrc`**.

You will also need colcon to build the package:
```sh
sudo apt install python3-colcon-common-extensions
```
Also, perform an install of [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (needed for ```wam_sim```). 
You will also need the `moveit` , `joint-state-controller` , `joint-trajectory-controller`  and `position-controllers` packages:
```sh
sudo apt-get install ros-melodic-moveit
sudo apt-get install ros-melodic-position-controllers
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-joint-trajectory-controller
```
**Do not source or add any of the ROS distros to `bashrc`**.

Also install [Libbarrett 2.0.0](https://git.barrett.com/software/libbarrett), which is a real-time controls library written in C++ that runs the WAM Arm.
## Installation
Clone the repo to your home directory:
```sh
cd ~
git clone https://git.barrett.com/software/barrett-ros2-pkg.git
```
Build the `barrett-ros2-pkg` and the `ros1_bridge ` packages:
```sh
cd ~/barrett-ros2-pkg/
sudo -s
./buildROS1.sh
exit
source ~/barrett-ros2-pkg/sourceROS1.sh
./build.sh
source ~/barrett-ros2-pkg/source.sh
cd ~/ros1-bridge-ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```
Source the `ros1_bridge` before running any of the launch files, or add it to `bashrc`:
```sh
echo 'source ~/ros1-bridge-ws/install/local_setup.bash' >> ~/.bashrc
exec "$BASH"
```

### Networking

This is required if you use run the ROS nodes across multiple computers. The below instructions are to setup the perception palm on the XWAM and to control it from a remote host.

In the host computer, open a new terminal and ssh into the XWAM
```
ssh summit@*xwam's ip address*
```
Enter the XWAM's password and open the hosts file
```
sudo vi /etc/hosts
```
Add the IP address of the remote host machine to the list and name it
```
*remote host's ip address* *remote host's name*
```
Save and exit it. Set the XWAM to be the ROS MASTER
```
export ROS_MASTER_URI=http://*XWAM's ip address*:11311
```
Open a new terminal and edit the host file in the host machine as above
```
sudo vi /etc/hosts
```
In the host machine, name the IP address of the xwam
```
*xwam's ip address* summit
```
Save and exit it. Update the ROS_MASTER as the XWAM in the host machine
```
export ROS_MASTER_URI=http://*XWAM's ip address*:11311
```

P.S: The ROS_MASTER_URI variable would be active as long as the terminal is open. If the terminal is closed and restarted, the ROS_MASTER_URI needs to be updated.
For more information on ROS network configuration refer to [ROS documentation](http://wiki.ros.org/ROS/NetworkSetup).

## Running WAM Simulation
**Note: The simulation requires a new terminal with ROS2 sourced first, and then ROS1. This is a requirement to run the ```ros1_bridge```.**

Source ROS2 and all the packages by running the source script:
```sh
source ~/barrett-ros2-pkg/source.sh
```
Source ROS1 and build all the packages by sourcing the `sourceROS1.sh`:
```sh
source ~/barrett-ros2-pkg/sourceROS1.sh
```
Launch the simulation: 
```sh
ros2 launch wam_sim wam_sim.launch.py dof:=<4/7> bhand:=<true/false> fts:=<true/false> mono_camera:=<true/false> stereo_camera:=<true/false>
```
**Note: The simulation will not work with `wam_node` running. If you wish to run `wam_node` and visualize it in simulation, run the empty simulation:**
```sh
source ~/barrett-ros2-pkg/source.sh
ros2 launch wam_sim wam_empty_sim.launch.py dof:=<4/7> bhand:=<true/false> fts:=<true/false>
```
Once the empty simulation launches, in the RVIZ window select File->Open Config and navigate to ~/barrett-ros2-pkg/wam_sim/config/, and choose wam_config.rviz.

## Running Standalone BarrettHand Simulation
**Note: The simulation requires a new terminal with ROS2 sourced first, and then ROS1. This is a requirement to run the ```ros1_bridge```.**

Source ROS2 and all the packages by running the source script:
```sh
source ~/barrett-ros2-pkg/source.sh
```
Source ROS1 and build all the packages by sourcing the `sourceROS1.sh`:
```sh
source ~/barrett-ros2-pkg/sourceROS1.sh
```
Launch the simulation: 
```sh
ros2 launch wam_sim wam_sim.launch.py bhand:=true fts:=<true/false>
```
**Note: The simulation will not work with `wam_node` running. If you wish to run `wam_node` and visualize it in simulation, run the empty simulation:**
```sh
source ~/barrett-ros2-pkg/source.sh
ros2 launch wam_sim wam_empty_sim.launch.py bhand:=true fts:=<true/false>
```
Once the empty simulation launches, in the RVIZ window select File->Open Config and navigate to ~/barrett-ros2-pkg/wam_sim/config/, and choose bhand_config.rviz.

## Running WAM Node
**Note: The WAM Node requires a new terminal with ROS2 sourced first, and then ROS1. This is a requirement to run the ```ros1_bridge```.**
Source ROS1 and ROS2 in a new terminal and launch ```wam_node``` **with the WAM Arm ,BarrettHand or Force/Torque Sensor connected via the CAN bus**:
```sh
source ~/barrett-ros2-pkg/source.sh
source ~/barrett-ros2-pkg/sourceROS1.sh
ros2 launch wam_node wam_node.launch.py
```

## Running Perception Palm

### Set up cameras:
1. **Connect the Perception Palm to the PC** before completing the following steps.

2. Edit the launch file to confirm camera setup parameters.
```
gedit ~/barrett-ros2-pkg/src/perception_palm/launch/perception_palm.launch
```
Ensure that the launch file targets the correct devices. By default, the two cameras are `/dev/video0` and `/dev/video2`. However, if you have other cameras on your system, this may be different. List the video devices with
    ```
    ls /dev/video*
    ```
    to see if you have extra video devices. To determine which devices are correct, you can use a program such as `guvcview`:
    ```
     sudo apt install guvcview
    guvcview -d /dev/video0
    ```
Check if running the command above with `dev/video0` and/or `dev/video2` shows output from the camera. If you need to change the default device(s), edit the lines in the launch file that look like this:
    ```
    <param name="device" type="string" value="/dev/video0" />
    ```
    

3. Load the correct camera module. For one camera
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```
    or two cameras
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128
```

*Notes*

Two cameras can be used simultaneously at a maximum resolution of 320 x 240 and a single camera can be used at a maximum resolution of 1600 x 1200. For information on maximum camera resolutions, refer to the spec sheet.

The camera with the red filter is physically installed with 180 degrees shift. So, in this configuration the camera with red filter is rotated by 180 degrees. Make sure that the appropriate camera (left/right) is shifted while configuring based on the corresponding device ennumerations (/dev/video0 or /dev/video2). The necessary changes can be made in the launch/perception_palm.launch file.

### Calibration

**IR Range finder**

**The IR range finder needs to be calibrated for the first time before using it.** It will work without calibration but it might not be accurate. Follow the instructions below to calibrate it. This step can be skipped if you do not want to use the IR or if the IR range finder is already calibrated.
```
cd ~/barrett-ros-pkg/src/perception_palm/include/MCP2210-Library
make
sudo ./dist/Debug/GNU-Linux-x86/ir_calibrate
```
Follow the onscreen instructions.

**Cameras**

Please refer to the ROS [Stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)/[Monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) calibration package. This step is completely optional. The cameras would work even if this step is skipped.

### Running the demo

1. Configure the cameras (every time you plug in the Perception Palm or reboot the computer). For one camera
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```
    or two cameras
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128
```

2. Become the root user to access the drivers and run the demo. For one camera:
```
sudo -s
source ~/barrett-ros2-pkg/source.sh
source ~/barrett-ros2-pkg/sourceROS1.sh
ros2 launch wam_node perception_palm_mono.launch.py
```
    or two cameras
```
sudo -s
source ~/barrett-ros2-pkg/source.sh
source ~/barrett-ros2-pkg/sourceROS1.sh
ros2 launch wam_node perception_palm_stereo.launch.py
```

4. To quit, press Ctrl-C. Then type `exit` to return to a regular terminal.

*Troubleshooting*

If the camera node fails to start in step 2, make sure the configuration you chose in step 1 matches the configuration in the launch file. See the "Set up cameras" section for details.


### Accessing the sensors

While the demo is running you can access the sensors from a separate terminal.
*Note: If you encounter a segmentation fault when launching rviz, make sure you're using the correct configuration file, then wait a moment and try again.*

#### LED

The LED can be turned off and on by calling the service barrett/palm/set_led_on.<br />
```	
rosservice call barrett/palm/set_led_on ['True']
rosservice call barrett/palm/set_led_on ['False']
```

#### Laser

The Laser can be turned off and on by calling the service barrett/palm/set_laser_on.<br />
```	
rosservice call barrett/palm/set_laser_on ['True']
rosservice call barrett/palm/set_laser_on ['False']
```

#### IR Range finder

The IR range finder publishes the range information as a sensor_msgs/Range message, at 1Hz to the topic barrett/palm/ir/range<br />
```
rostopic echo barrett/palm/ir/range
```

#### Camera

By default, the two camera feeds are published at the rate of 30 fps with a resolution of 320x240 px. They are published to the topics, barrett/palm/left/image_raw and barrett/palm/right/image_Raw.<br />
While using the monocular camera mode, the images are published at the rate of 30 fps with a resolution of 1600 x 1200 to the topic barrett/palm/image_raw.<br />


### Troubleshooting

Trying to restart the perception_palm package multiple times might fail with an error "Error setting SPI Parameters".
This can be solved by unplugging and plugging back the USB to the port.

## Running WAM Demos
##### Teach and Play
Run either ```wam_node``` or ```wam_sim``` as described above.

**To teach**, in a new terminal, source ROS1 and the ```wam_demos``` package, and run the ```teach``` executable:
```sh
source ~/barrett-ros2-pkg/sourceROS1.sh
rosrun wam_demos teach -n <bag-name> -t <record-type>
```
The ```-n <bag-name>``` field is optional and can be used to specify a name for the saved rosbag. The ```-t <record_type>``` field is optional and can be used to specify the record type for the saved trajectory. Possible values are:
- **jp**: Record trajectory using joint positions
- **jv**: Record trajectory using joint velocities
- **tp**: Record trajectory using tool poses
- **tv**: Record trajectory using tool velocities
 
The rosbags are saved in the ```wam_rosbags``` folder.

**To play**, source ROS1 and the ```wam_demos``` package and run the ```play``` executable:
```sh
source ~/barrett-ros2-pkg/sourceROS1.sh
rosrun wam_demos play <bag-name>
```
## Subscribed Topics:
To check all the available Topics, run either ```wam_node``` or ```wam_sim``` as described above, and run ```ros2 topic list```

### Wam Subscribed Topics:
* **`/wam/RTJointPositionCMD`** **Type**: [*wam_msgs/RTJointPositions*](wam_msgs/msg/RTJointPositions.msg)

    Topic to command continuous joint positions to the WAM.
* **`/wam/RTLinearandAngularVelocityCMD`** **Type**: [*wam_msgs/RTLinearandAngularVelocity*](wam_msgs/msg/RTLinearandAngularVelocity.msg)

    Topic to command continuous linear and angular velocities to the end-effector of the WAM. 
* **`/wam/RTAngularVelocityCMD`** **Type**: [*wam_msgs/RTAngularVelocity*](wam_msgs/msg/RTAngularVelocity.msg)

    Topic to command continuous angular velocities to the end-effector of the WAM.
* **`/wam/RTLinearVelocityCMD`** **Type**: [*wam_msgs/RTLinearVelocity*](wam_msgs/msg/RTLinearVelocity.msg)

    Topic to command continuous linear velocities to the end-effector of the WAM.
* **`/wam/RTCartPositionCMD`** **Type**: [*wam_msgs/RTCartPosition*](wam_msgs/msg/RTCartPosition.msg)

    Topic to command continuous end-effector cartesian positions to the WAM. 
* **`/wam/RTCartPoseCMD`** **Type**: [*wam_msgs/RTCartPose*](wam_msgs/msg/RTCartPose.msg)

    Topic to command continuous end-effector cartesian poses to the WAM.
* **`/wam/RTCartOrientationCMD`** **Type**: [*wam_msgs/RTCartOrientation*](wam_msgs/msg/RTCartOrientation.msg)

 Topic to command continuous end-effector cartesian orientations to the WAM.
* **`/wam/RTJointVelocityCMD`** **Type**: [*wam_msgs/RTJointVelocities*](wam_msgs/msg/RTJointVelocities.msg)

    Topic to command continuous joint velocities to the WAM.

## Published Topics

To check all the available Topics, run either ```wam_node``` or ```wam_sim``` as described above, and run ```ros2 topic list```

### WAM Published Topics:
* **`/wam/ToolPose`** **Type**: [*geometry_msgs/PoseStamped*](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)

    Current end-effector pose of the WAM 
* **`/joint_states`** **Type**: [*sensor_msgs/JointState*](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)

    Current joint positions of the WAM and/or BarrettHand
* **`/wam/jointVelocity`** **Type**: [*sensor_msgs/JointState*](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)

    Current joint velocity of the WAM 
* **`/wam/toolVelocity`** **Type**: [*geometry_msgs/Twist*](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)

    Current tool velocity of the WAM.

### FTS Published Topics:
* **`/FTS/States`** **Type**: [*geometry_msgs/Wrench*](https://docs.ros.org/api/geometry_msgs/html/msg/Wrench.html)

    Data from the Force/Torque sensor.
    
### BarrettHand Published Topics:
* **`/bhand/TactileStates`** **Type**: [*bhand_msgs/TactileStateArray*](bhand_msgs/msg/TactileStateArray.msg) and [*bhand_msgs/TactileState*](bhand_msgs/msg/TactileState.msg)

    Data from BarrettHand Tactile Sensors.
* **`/bhand/FingertipTorques`** **Type**: [*bhand_msgs/FingerTipTorques*](bhand_msgs/msg/FingerTipTorques.msg)

    Data from BarrettHand FingerTip Torque sensors.

### Perception Palm Published Topics:
* **`/perception_palm/ir/range`** **Type**: [*sensor_msgs/Range*](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html)

    Data from the perception palm IR sensors. Range value goes goes from 0.025m and 0.5m.
* **`/perception_palm/left/image_raw`** **Type**: [*sensor_msgs/Image*](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)

    **Only available with stereo configuration of pereption palm:** Image data from the perception palm left camera. This image is flipped 180 degrees and has a red filter on it. Resolution of the image is 320 x 240 at 30FPS.
* **`/perception_palm/right/image_raw`** **Type**: [*sensor_msgs/Image*](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html)

    **Only available with stereo configuration of pereption palm:** Image data from the perception palm right camera. Resolution of the image is 320 x 240 at 30FPS.
* **`/perception_palm/image_raw`** **Type**: [*sensor_msgs/Image*](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html)

    **Only available with mono configuration of pereption palm:** Image data from the perception palm camera. Resolution of the image is 1600 x 1200 at 30FPS.
    
## Services

### Wam Services:
To check all the available services, run either ```wam_node``` or ```wam_sim``` as described above, and run ```ros2 service list```

* **`/wam/idle`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to command the WAM to idle. This terminates the position controller (if active).<br/>
    **Service call for ROS2:** `ros2 service call /wam/idle std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /wam/idle`
    
* **`/wam/moveHome`** **Type**:  [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to command the WAM to it's home position. Returns `true` on success and `false` on failure.<br/>
    **Service call for ROS2:** `ros2 service call /wam/moveHome std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /wam/moveHome`

* **`/wam/holdCartPosition`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to command the WAM to hold it's current end-effector cartesian position. The request field can be set to `false` to deactivate an existing hold, or `true` to activate one.<br/>
    **Service call for ROS2:** `ros2 service call /wam/holdCartPosition std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /wam/holdCartPosition "data: true"`
* **`/wam/holdJointPosition`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to command the WAM to hold it's current joint position. The request field can be set to `false` to deactivate an existing hold, or `true` to activate one.<br/>
    **Service call for ROS2:** `ros2 service call /wam/holdJointPosition std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /wam/holdJointPosition "data: true"`
* **`/wam/holdCartOrientation`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to command the WAM to hold it's current end-effector cartesian orientation. The request field can be set to `false` to deactivate an existing hold, or `true` to activate one.<br/>
    **Service call for ROS2:** `ros2 service call /wam/holdCartOrientation std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /wam/holdCartOrientation "data: true"`
* **`/wam/holdCartPose`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to command the WAM to hold it's current end-effector cartesian pose. The request field can be set to `false` to deactivate an existing hold, or `true` to activate one.<br/>
    **Service call for ROS2:** `ros2 service call /wam/holdCartPose std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /wam/holdCartPose "data: true"`
* **`/wam/gravityCompensate`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to command the WAM to activate or deactivate gravity compensation.<br/>
    **Service call for ROS2:** `ros2 service call /wam/gravityCompensate std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /wam/gravityCompensate "data: true"`
* **`/wam/setVelocityLimit`** **Type**: [*wam_srvs/VelocityLimit*](wam_srvs/srv/VelocityLimit.srv)

    Service to set a joint velocity limit for the **safety system** of the WAM. Default value: 1rad/s.<br/>
    **Service call for ROS2:** `ros2 service call /wam/setVelocityLimit wam_srvs/srv/VelocityLimit "velocity_limit: 1" `<br/>
    **Service call for ROS1:** `rosservice call /wam/setVelocityLimit "velocity_limit: 1"`
* **`/wam/moveToRandomJointPosition`** **Type**: [*std_srvs/Empty*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)

    Only available for `wam_sim`. Moves WAM to a random joint position.<br/>
    
    **Service call for ROS2:** `ros2 service call /wam/moveToRandomJointPosition std_srvs/srv/Empty`<br/>
    **Service call for ROS1:** `rosservice call /wam/moveToRandomJointPosition "{}" `
* **`/wam/moveToJointPosition`** **Type**: [*wam_srvs/JointMove*](wam_srvs/srv/JointMove.srv)

    Service to command a single goal joint position to the WAM. Returns `true` on success and `false` on failure.<br/>
    **Service call for ROS2:**<br/>
    ```sh
    ros2 service call /wam/moveToJointPosition wam_srvs/srv/JointMove "joint_state:
      header:
          stamp: 
            sec: 0 
            nanosec: 0
          frame_id: ''
      name: ['']
      position: [0, 0, 0, 0, 0, 0, 0]
      velocity: [0]
      effort: [0]" 
    ```
        
    **Service call for ROS1:**<br/>
    ```sh
    rosservice call /wam/moveToJointPosition "joint_state:
        header:
            seq: 0
            stamp: {secs: 0, nsecs: 0}
            frame_id: ''
        name: ['']
        position: [0, 0, 0, 0, 0, 0, 0]
        velocity: [0]
        effort: [0]" 
    ```
* **`/wam/moveToCartOrientation`** **Type**: [*wam_srvs/CartOrientationMove*](wam_srvs/srv/CartOrientationMove.srv)

    Service to command a single goal end-effector cartesian orientation to the WAM. Returns `true` on success and `false` on failure.<br/>
    **Service call for ROS2:**<br/>
    ```sh
    ros2 service call /wam/moveToCartOrientation wam_srvs/srv/CartOrientationMove "orientation:
  	    x: 0.0
  	    y: 0.0
  	    z: 0.0
  	    w: 0.0" 

    ```
        
    **Service call for ROS1:**<br/>
    ```sh
    rosservice call /wam/moveToCartOrientation "orientation:
  	    x: 0.0
  	    y: 0.0
  	    z: 0.0
  	    w: 0.0" 

    ```
* **`/wam/moveToCartPosition`** **Type**: [*wam_srvs/CartPositionMove*](wam_srvs/srv/CartPositionMove.srv)

    Service to command a single goal end-effector cartesian position to the WAM. Returns `true` on success and `false` on failure.<br/>
    **Service call for ROS2:**<br/>
    ```sh
    ros2 service call /wam/moveToCartPosition wam_srvs/srv/CartPositionMove "position:
  	  x: 0.0
  	  y: 0.0
  	  z: 0.0" 
    ```
        
    **Service call for ROS1:**<br/>
    ```sh
    rosservice call /wam/moveToCartPosition "position:
      x: 0.0
      y: 0.0
      z: 0.0" 
    ```
* **`/wam/moveToCartPose`** **Type**: [*wam_srvs/CartPoseMove*](wam_srvs/srv/CartPoseMove.srv)

    Service to command a single goal end-effector cartesian pose to the WAM. Returns `true` on success and `false` on failure.<br/>
    **Service call for ROS2:**<br/>
    ```sh
    ros2 service call /wam/moveToCartPose wam_srvs/srv/CartPoseMove "pose:
	    position:
    	  x: 0.0
    	  y: 0.0
    	  z: 0.0
  	    orientation:
    	  x: 0.0
    	  y: 0.0
    	  z: 0.0
    	  w: 0.0" 
    ```
        
    **Service call for ROS1:**<br/>
    ```sh
    rosservice call /wam/moveToCartPose "pose:
  	    position:
    	  x: 0.0
    	  y: 0.0
    	  z: 0.0
  	    orientation:
    	  x: 0.0
    	  y: 0.0
    	  z: 0.0
    	  w: 0.0" 
    ```
### FTS Services:
**`/FTS/Tare`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to tare the Force/Torque Sensor.<br/>
    **Service call for ROS2:** `ros2 service call /FTS/Tare std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /FTS/Tare`
    
### BarrettHand Services:
* **`/bhand/idle`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to command the BarrettHand to idle. This terminates the position controller (if active).<br/>
    **Service call for ROS2:** `ros2 service call /bhand/idle std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /bhand/idle`
* **`/bhand/closeGrasp`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to close the BarrettHand grasp.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/closeGrasp std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /bhand/closeGrasp`
* **`/bhand/openGrasp`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to open the BarrettHand grasp.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/openGrasp std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /bhand/openGrasp`
* **`/bhand/openSpread`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to open the BarrettHand Spread.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/openSpread std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /bhand/openSpread`
* **`/bhand/closeSpread`** **Type**: [*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)

    Service to close the  BarrettHand Spread.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/closeSpread std_srvs/srv/Trigger `<br/>
    **Service call for ROS1:** `rosservice call /bhand/closeSpread`

* **`/bhand/moveToFingerPositions`** **Type**: [*bhand_srvs/FingerPosition*](bhand_srvs/srv/FingerPosition.srv)

    Service to move the BarrettHand fingers to commanded position.<br/>
    **Service call for ROS2:**<br/>
    ```sh
    ros2 service call /bhand/moveToFingerPositions bhand_srvs/srv/FingerPosition "position:
	- 0
	- 0
	- 0" 
    ```
        
    **Service call for ROS1:**<br/>
    ```sh
    rosservice call /bhand/moveToFingerPositions "position:
	- 0
	- 0
	- 0" 
    ```
* **`/bhand/moveToGraspPosition`** **Type**: [*bhand_srvs/GraspPosition*](bhand_srvs/srv/GraspPosition.srv)

    Service to move the BarrettHand grasp to desired position.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/moveToGraspPosition bhand_srvs/srv/GraspPosition "position: 0.0"`<br/>
    **Service call for ROS1:** `rosservice call /bhand/moveToGraspPosition "position: 0.0"`
* **`/bhand/moveToSpreadPosition`** **Type**: [*bhand_srvs/SpreadPosition*](bhand_srvs/srv/SpreadPosition.srv)

    Service to move the BarrettHand spread to desired position.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/moveToSpreadPosition bhand_srvs/srv/SpreadPosition "position: 0.0"`<br/>
    **Service call for ROS1:** `rosservice call /bhand/moveToSpreadPosition "position: 0.0"`
* **`/bhand/moveToFingerVelocities`** **Type**: [*bhand_srvs/FingerVelocity*](bhand_srvs/srv/FingerVelocity.srv)

    Service to command the BarrettHand Fingers to move at a desired velocity. The fingers move until they reach their joint limits, or until **`bhand/idle`** is called.<br/>
    **Service call for ROS2:**<br/>
    ```sh
    ros2 service call /bhand/moveToFingerVelocities bhand_srvs/srv/FingerVelocity "velocity:
	- 0
	- 0
	- 0" 
    ```
        
    **Service call for ROS1:**<br/>
    ```sh
    rosservice call /bhand/moveToFingerVelocities "velocity:
	- 0
	- 0
	- 0" 
    ``` 
* **`/bhand/moveToGraspVelocity`** **Type**: [*bhand_srvs/GraspVelocity*](bhand_srvs/srv/GraspVelocity.srv)

    Service to command the BarrettHand Grasp to move at a desired velocity. The grasp moves until they reach their joint limits, or until **`bhand/idle`** is called.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/moveToGraspVelocity bhand_srvs/srv/GraspVelocity "velocity: 0.0"`<br/>
    **Service call for ROS1:** `rosservice call /bhand/moveToGraspVelocity "velocity: 1.0"`
* **`/bhand/moveToSpreadVelocity`** **Type**: [*bhand_srvs/SpreadVelocity*](bhand_srvs/srv/SpreadVelocity.srv)

    Service to command the BarrettHand Spread to move at a desired velocity. The spread moves until they reach their joint limits, or until **`bhand/idle`** is called.<br/>
    **Service call for ROS2:** `ros2 service call /bhand/moveToSpreadVelocity bhand_srvs/srv/SpreadVelocity "velocity: 0.0"`<br/>
    **Service call for ROS1:** `rosservice call /bhand/moveToSpreadVelocity "velocity: 1.0"`

### Perception Palm Services:
* **`/barrett/palm/set_laser_on`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to turn on/off Perception Palm laser projector.<br/>
    **Service call for ROS2:** `ros2 service call /barrett/palm/set_laser_on std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /barrett/palm/set_laser_on "data: true"`
* **`/barrett/palm/set_led_on`** **Type**: [*std_srvs/SetBool*](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html)

    Service to turn on/off Perception Palm led.<br/>
    **Service call for ROS2:** `ros2 service call /barrett/palm/set_led_on std_srvs/srv/SetBool "data: true" `<br/>
    **Service call for ROS1:** `rosservice call /barrett/palm/set_led_on "data: true"`
    
## Directory Structure

	├── bhand_msgs                                  #ROS2 package with Bhand messages.
    │   ├── msg
    │   │   ├── FingerTipTorques.msg
    │   │   ├── TactileStateArray.msg
    │   │   └── TactileState.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── bhand_srvs                                  #ROS2 package with Bhand service files
    │   ├── srv
    │   │   ├── FingerPosition.srv
    │   │   ├── FingerVelocity.srv
    │   │   ├── GraspPosition.srv
    │   │   ├── GraspVelocity.srv 
    │   │   ├── SpreadPosition.srv
    │   │   └── SpreadVelocity.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── docs
    ├── scripts
    │   ├── build
    │   ├── debug
    │   ├── run
    │   ├── setup
    │   └── test
    ├── src                                         #Directory with ROS1 packages
    │   ├── bhand_msgs                              #ROS1 package with Bhand message files
    │   │   ├── msg
    │   │   │   ├── FingerTipTorques.msg
    │   │   │   ├── TactileStateArray.msg
    │   │   │   └── TactileState.msg
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── bhand_srvs	                            #ROS1 package with bhand Service files
    │   │   ├── srv
    │   │   │   ├── FingerPosition.srv
    │   │   │   ├── FingerVelocity.srv
    │   │   │   ├── GraspPosition.srv
    │   │   │   ├── GraspVelocity.srv
    │   │   │   ├── SpreadPosition.srv
    │   │   │   └── SpreadVelocity.srv
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── perception_palm                         #Existing ROS1 perception palm package 
                                                    (ported from barrett-ros-pkg) modified to use 
                                                    standard ROS messages and services files.
    │   │   ├── include
    │   │   │   ├── MCP2210-Library
    │   │   │   └── palm_mcp.h
    │   │   ├── launch
    │   │   │   ├── palm_mono.rviz
    │   │   │   ├── palm_stereo.rviz
    │   │   │   └── perception_palm.launch
    │   │   ├── src
    │   │   │   └── palm_mcp.cpp
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── wam_demos                           #ROS1 WAM demos package. Uses ROSBAGS to record and
                                                play back  trajectories in wam_sim or wam_node. 
                                                Uses ros1_bridge to communicate with wam_node, 
                                                which introduces delay.
    │   │   ├── src
    │   │   │   ├── play.cpp
    │   │   │   ├── teach.cpp
    │   │   │   └── teach.h
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── wam_description             #ROS1 package with WAM and Bhand mesh and URDF files.
    │   │   ├── meshes
    │   │   ├── urdf
    │   │   │   ├── 4dof_link.urdf.xacro             #URDF for WAM 4DOF outer link.
    │   │   │   ├── barrett_hand.urdf.xacro          #BarrettHand URDF.
    │   │   │   ├── common.urdf.xacro                #URDF with common macros defined.
    │   │   │   ├── fts.urdf.xacro                   #Force/Torque sensor URDF.
    │   │   │   ├── mono_camera.urdf.xacro           #Perception Palm mono camera URDF.
    │   │   │   ├── palm_tactile_sensors.urdf.xacro  #BarrettHand palm tactile sensor URDF.
    │   │   │   ├── stereo_camera.urdf.xacro         #Perception Palm stereo camera URDF.
    │   │   │   ├── transmission.urdf.xacro          #URDF with WAM and BarrettHand transmissions 
                                                     required for ROS control with Gazebo.
    │   │   │   ├── wam_base.urdf.xacro              #WAM URDF without outer link.
    │   │   │   ├── wam.urdf.xacro                   #Main URDF file that interfaces all other 
                                                     URDF files. Loads WAM/BarrettHand config 
                                                     based on arguments passed to it.
    │   │   │   └── wrist_link.urdf.xacro            #URDF with 7DOF WAM outer link
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── wam_moveit_config                       #Moveit configuration package for WAM. 
                                                    Contains launch files for gazebo, RIVZ, 
                                                    moveit and all necessary controllers for 
                                                    WAM/BarrettHand control.
    │   │   ├── config                              #Contains moveit configuration files for all 
                                                    WAM/Barrett Hand configurations.
    │   │   ├── launch                              #Moveit launch files
    │   │   │   ├── gazebo.launch
    │   │   │   ├── move_group.launch
    │   │   │   ├── ompl_planning_pipeline.launch.xml
    │   │   │   ├── planning_context.launch
    │   │   │   ├── ros_controllers.launch
    │   │   │   ├── trajectory_execution.launch.xml
    │   │   │   ├── wam4dof_moveit_controller_manager.launch.xml
    │   │   │   └── wam_moveit.launch               #Main moveit launch file. Launches all 
                                                    other launch files, RVIZ and gazebo.
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── wam_msgs                                #ROS1 package with WAM messages
    │   │   ├── msg
    │   │   │   ├── RTAngularVelocity.msg
    │   │   │   ├── RTCartOrientation.msg
    │   │   │   ├── RTCartPose.msg
    │   │   │   ├── RTCartPosition.msg
    │   │   │   ├── RTJointPositions.msg
    │   │   │   ├── RTJointVelocities.msg
    │   │   │   ├── RTLinearandAngularVelocity.msg
    │   │   │   └── RTLinearVelocity.msg
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── wam_sim_node                            #ROS1 package that controls simulation.
    │   │   ├── launch
    │   │   │   └── wam_sim_node.launch             #Launches simulation node for different wam 
                                                    configurations based on passed arguments
    │   │   ├── src 
    │   │   │   ├── bhand_sim_node.cpp              #Bhand node for standalone hand simulation   
    │   │   │   ├── bhand_sim_node.h                #Contains Publishers and Service and 
                                                    Subscriber callbacks for Bhand simulation.
    │   │   │   ├── gazebo_publisher.h              #Converts gazebo messages to ROS messages
                                                    requried for tactile sensor simulation
    │   │   │   ├── wam_sim_node.cpp                #WAM simulation node.
    │   │   │   └── wam_sim_node.h                  #Contains Publishers and Service 
                                                    and Subscriber callbacksfor WAM simulation.
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   ├── wam_srvs                                #ROS1 package with WAM services.
    │   │   ├── srv
    │   │   │   ├── CartOrientationMove.srv
    │   │   │   ├── CartPoseMove.srv
    │   │   │   ├── CartPositionMove.srv
    │   │   │   ├── JointMove.srv
    │   │   │   └── VelocityLimit.srv
    │   │   ├── CMakeLists.txt
    │   │   └── package.xml
    │   └── CMakeLists.txt
    ├── wam_demos                       #WIP (ISSUE #10) ROS2 wam demos package. Records and plays back 
                                        trajectories saved to csv files, since ROS2 ROSBAGS are not yet fully complete.
                                        Uses bridging for communication with wam_sim. Improves performance over 
                                        ROS1 wam_demos.
    │   ├── src
    │   │   ├── play.cpp
    │   │   ├── teach.cpp
    │   │   └── teach.h
    │   ├── CMakeLists.txt
    │   └── package.xml 
    ├── wam_msgs                                    #ROS2 package with WAM messages.
    │   ├── msg
    │   │   ├── RTAngularVelocity.msg
    │   │   ├── RTCartOrientation.msg
    │   │   ├── RTCartPose.msg
    │   │   ├── RTCartPosition.msg
    │   │   ├── RTJointPositions.msg
    │   │   ├── RTJointVelocities.msg
    │   │   ├── RTLinearandAngularVelocity.msg
    │   │   └── RTLinearVelocity.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── wam_node                                    #ROS2 wam node. Controls WAM hardware.
    │   ├── launch                                  #Contains launch files for wam node and 
                                                    perception palm.
    │   │   ├── perception_palm_mono.launch.py      #Launches ros1_bridge and ROS1 
                                                    perception palm package as a python 
                                                    subprocess for mono camera configuration.
    │   │   ├── perception_palm_stereo.launch.py    #Launches ros1_bridge and ROS1
                                                    Perception palm package as a python
                                                    subprocess for stereo configuration.
    │   │   └── wam_node.launch.py                  #Launches wam_node and ros1_bridge
    │   ├── src
    │   │   ├── bhand_publishers.h              #Contains ROS2 Publisher node for 
                                                Bhand joint states and sensor data.
    │   │   ├── bhand_services.h                #Contains ROS2 Service servers 
                                                for Bhand services
    │   │   ├── custom_systems.h                #Defines custom Libbarrett 
                                                systems required for wam_node
    │   │   ├── fts_node.h                      #Contains Publishers and Services for the FTS                                               
    │   │   ├── wam_node.cpp                    #Launches any configuration for 
                                                publishers, subscribers and service 
                                                servers depending on configuration 
                                                of WAM or standalone BarrettHand.
    │   │   ├── wam_publishers.h                #Publishes wam joint states and sensor data.
    │   │   ├── wam_services.h                  #Contains service servers for WAM.
    │   │   └── wam_subscribers.h               #Contains subscribers for real-time control.
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── wam_sim                             #ROS2 Package that launches ROS1 WAM simulation. 
                                            Also contains empty simulation launch files, 
                                            used for RVIZ simulation with WAM hardware connected.
    │   ├── config
    │   │   ├── bhand_config.rviz
    │   │   └── wam_config.rviz
    │   ├── launch
    │   │   ├── wam4dof
    │   │   │   ├── meshes
    │   │   │   ├── wam4dof_fts.urdf
    │   │   │   ├── wam4dof_hand_fts.urdf
    │   │   │   ├── wam4dof_hand.urdf
    │   │   │   └── wam4dof.urdf
    │   │   ├── wam7dof
    │   │   │   ├── meshes
    │   │   │   ├── barrett_hand.urdf
    │   │   │   ├── wam7dof_fts_hand.urdf
    │   │   │   ├── wam7dof_fts.urdf
    │   │   │   ├── wam7dof_hand_fts.urdf
    │   │   │   ├── wam7dof_hand.urdf
    │   │   │   └── wam7dof.urdf
    │   │   ├── wam_empty_sim.launch.py
    │   │   └── wam_sim.launch.py
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── wam_config.rviz
    ├── wam_srvs                                    #ROS2 package with WAM services
    │   ├── srv
    │   │   ├── CartOrientationMove.srv
    │   │   ├── CartPoseMove.srv
    │   │   ├── CartPositionMove.srv
    │   │   ├── JointMove.srv
    │   │   └── VelocityLimit.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── buildROS1.sh                                #Script to build all ROS1 packages
    ├── build.sh                                    #Script to build all ROS2 packages
    ├── CHANGELOG.md
    ├── clean.sh
    ├── CONTRIBUTING.md
    ├── LICENSE
    ├── README.md
    ├── sourceROS1.sh                               #Script to source ROS1 workspace
    ├── source.sh                                   #Script to source ROS2 packages
    └── VERSION

## Contributing
Please see [CONTRIBUTING.md](CONTRIBUTING.md)

## Resources

[moveit](http://wiki.ros.org/moveit)

