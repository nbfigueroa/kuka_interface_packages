#!/bin/bash

source ~/.bashrc
rtk_pkg_tools_path=$(rospack find rtk_pkg_tools)
cd $rtk_pkg_tools_path/..

if [ $# -gt 0 ]; then
    echo "You choose to run the bridge for the $1 arm"
    sudo -E ./bin/LWRMain --config packages/kuka_fri_bridge/KUKABridge_${1}
else
    echo "Running bridge with no arguments, meaning you will use default values (name:KUKA, arm: Right)."
    sudo -E ./bin/LWRMain --config packages/kuka_fri_bridge/KUKABridge
fi


