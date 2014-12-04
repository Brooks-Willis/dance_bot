#!/usr/bin/env bash


source catkin_utilities.sh


install_camera_calibration() {
    rosdep install camera_calibration
    rosmake camera_calibration
}


check_camera_topics() {
    topics=$(rostopic list)
    if [[ $topics != *"/camera/camera_info"* ]]; then
        echo "Error, line" ${LINENO} ": /camera/camera_info topic is not being published. Make sure to execute run_gscam.sh before running this file."
        exit 1
    fi

    if [[ $topics != *"/camera/image_raw"* ]]; then
        echo "Error, line" ${LINENO} ": /camera/image_raw topic is not being published. Make sure to execute run_gscam.sh before running this file."
        exit 1
    fi
}


run_camera_calibration() {
    # See http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration for instructions
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/
}


copy_calibration_file() {
    cp /home/$USER/.ros/camera_info/camera.yaml /home/$USER/.ros/camera_info/camera_calibration.yaml --remove-destination
}


install_camera_calibration
check_camera_topics
run_camera_calibration
copy_calibration_file
