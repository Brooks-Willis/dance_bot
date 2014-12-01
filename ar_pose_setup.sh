#!/usr/bin/env bash


source catkin_utilities.sh


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
