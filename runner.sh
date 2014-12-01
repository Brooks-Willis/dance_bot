#!/usr/bin/env bash

./ar_pose_setup.sh
./run_gscam.sh
./camera_calibration.sh
./uvc_camera_setup.sh

# roslaunch ar_pose ar_pose_multi_gopro.launch
