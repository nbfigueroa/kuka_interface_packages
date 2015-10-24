#!/bin/bash
source ~/.bashrc
rtk_pkg_tools_path=$(rospack find rtk_pkg_tools)
cd $rtk_pkg_tools_path/..
sudo -E ./bin/LWRMain --config packages/kuka_fri_bridge/KUKAMirror
