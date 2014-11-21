#!/usr/bin/env bash

catkin_root() {
    catkin_ws=$(find ~ -type d -name "catkin_ws")
    cd $catkin_ws
}

catkin_src() {
    catkin_root
    cd src
}

download_ar_pose() {
    if [ ! -d "ar_tools" ]; then
        git clone git@github.com:arlolinscope/ar_tools.git
    fi
}

setup_ar_pose() {
    catkin_src
    cd ar_tools
    rosmake
}

catkin_src
download_ar_pose
setup_ar_pose
catkin_root
catkin_make
