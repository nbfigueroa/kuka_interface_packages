#!/bin/bash
source ~/.bashrc
roscd rtk_pkg_tools
cd ../
#cd /home/nbfigueroa/catkin_ws/src/robot-toolkit
sudo -E ./bin/LWRMain --config packages/rtk_mirror/KUKAMirror
