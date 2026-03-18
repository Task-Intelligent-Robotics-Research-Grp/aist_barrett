#!/bin/bash
cd wam_msgs
rm -r build
rm -r install
rm -r log

cd ../wam_srvs
rm -r build
rm -r install
rm -r log

cd ../bhand_msgs
rm -r build
rm -r install
rm -r log

cd ../bhand_srvs
rm -r build
rm -r install
rm -r log

cd ../wam_node
rm -r build
rm -r install
rm -r log

cd ../wam_sim
rm -r build
rm -r install
rm -r log

cd ../wam_demos
rm -r build
rm -r install
rm -r log

cd ..
rm -r build
rm -r devel
rm -r install
rm -r log
