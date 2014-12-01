#!/usr/bin/env bash


install_uvc_camera() {
    version=$(rosversion -d)

    if [[ $version == "hydro" ]]
    then
        sudo apt-get install ros-hydro-uvc-camera
    elif [[ $version == "indigo" ]]
    then
        sudo apt-get install ros-indigo-uvc-camera
    fi
}


create_camera_calibration_file() {
    version=$(rosversion -d)
    sudo cp /home/$USER/.ros/camera_info/camera.yaml /opt/ros/$version/share/uvc_camera/camera_calibration.yaml
    sudo chown $USER:$USER /opt/ros/$version/share/uvc_camera/camera_calibration.yaml
}


install_uvc_camera
create_camera_calibration_file
