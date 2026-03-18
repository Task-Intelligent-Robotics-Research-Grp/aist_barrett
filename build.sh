#!/bin/bash
source /opt/ros/dashing/setup.bash

cd wam_srvs
colcon build
source install/setup.bash

cd ../wam_msgs
colcon build
source install/setup.bash

cd ../bhand_srvs
colcon build
source install/setup.bash

cd ../bhand_msgs
colcon build
source install/setup.bash

cd ../wam_node
colcon build

cd ../wam_demos
colcon build

cd ../wam_sim
colcon build
cd ..
