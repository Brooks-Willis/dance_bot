#!/usr/bin/env bash


source catkin_utilities.sh


free_video_input() {
    video_input=$(cat video_device.txt)
    fuser $video_input --kill --silent
}


copy_fiducial_data() {
    catkin_ws="$(find ~ -type d -name catkin_ws)"
    fiducial_folder="$(find ~ -type d -name fiducial_tracking)/data"
    ar_pose_directory="$(find $catkin_ws -type d -name ar_pose -print -quit)/data/dance_bot_data"
    echo $ar_pose_directory
    cp --recursive $fiducial_folder $ar_pose_directory
}   


create_launch_file() {
    video_input="$(cat video_device.txt)"
    fiducials_folder="$(find ~ -type d -name fiducial_tracking)/data"
    catkin_src
    cd ar_tools/ar_pose/launch
    data="<launch>
              <remap to=\"/camera/set_camera_info\" from=\"/set_camera_info\" />
              <node pkg=\"rviz\" type=\"rviz\" name=\"rviz\" args=\"-d \$(find ar_pose)/launch/live_multi.rviz\"/>
              <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"world_to_cam\" args=\"0 0 0.5 -1.57 0 -1.57 world camera 1\" />
              <node ns=\"camera\" pkg=\"image_proc\" type=\"image_proc\" name=\"image_proc\"/>
              <node pkg=\"gscam\" type=\"gscam\" name=\"gscam\" output=\"screen\">
                  <param name=\"camera_name\" type=\"string\" value=\"camera\" />
                  <param name=\"frame_id\" type=\"string\" value=\"camera\" />
                  <param name=\"gscam_config\" type=\"string\" value=\"v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1,width=1920,height=1080 ! ffmpegcolorspace\" />
                  <param name=\"camera_info_url\" type=\"string\" value=\"file://\$(env HOME)/.ros/camera_info/camera_calibration.yaml\" />
              </node>
              <node name=\"ar_pose\" pkg=\"ar_pose\" type=\"ar_multi\" respawn=\"false\" output=\"screen\">
                  <param name=\"marker_pattern_list\" type=\"string\" value=$fiducials_folder/>
                  <param name=\"threshold\" type=\"int\" value=\"100\"/>
              </node>
          </launch>"
    echo $data > ar_pose_dance_bot.launch
}

free_video_input
create_launch_file
copy_fiducial_data
roslaunch ar_pose ar_pose_dance_bot.launch
