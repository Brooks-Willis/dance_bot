#!/usr/bin/env bash


source catkin_utilities.sh


free_video_input() {
    video_input=$(cat video_device.txt)
    fuser $video_input --kill --silent
}


create_launch_file() {
    video_input=$(cat video_device.txt)
    catkin_src
    cd ar_tools/ar_pose/launch
    data=$'<launch>\n
            <remap to="/camera/set_camera_info" from="/set_camera_info" />\n
            <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ar_pose)/launch/live_multi.rviz"/>\n
            <node pkg="tf" type="static_transform_publisher" name="world_to_cam" args="0 0 0.5 -1.57 0 -1.57 world camera 1" />\n
            <node ns="camera" pkg="image_proc" type="image_proc" name="image_proc"/>\n
            <node pkg="gscam" type="gscam" name="gscam" output="screen">\n
                <param name="camera_name" type="string" value="camera" />\n
                <param name="frame_id" type="string" value="camera" />\n
                <param name="gscam_config" type="string" value="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1,width=1920,height=1080 ! ffmpegcolorspace" />\n
                <param name="camera_info_url" type="string" value="file://$(env HOME)/.ros/camera_info/camera_calibration.yaml" />\n
            </node>\n
            <node name="ar_pose" pkg="ar_pose" type="ar_multi" respawn="false" output="screen">\n
                <param name="marker_pattern_list" type="string" value="$(find ar_pose)/data/multi/object_letters"/>\n
                <param name="threshold" type="int" value="100"/>\n
            </node>\n
          </launch>\n'
    echo $data > ar_pose_dance_bot.launch
}

free_video_input
create_launch_file
roslaunch ar_pose ar_pose_dance_bot.launch