#!/usr/bin/env bash


launch_gscam() {
    video_device=$(cat video_device.txt)
    export GSCAM_CONFIG="v4l2src device=$video_device ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
    rosrun gscam gscam
}


launch_gscam "video0"
