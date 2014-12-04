# Everything you need to track multiple fiducials in ROS

Please note that this setup process clobbers existing camera calibration files in target folders. If you have camera calibration files that you want to keep, do not run the setup scripts without first modifying them.

This setup process assumes that you already have ROS Hydro installed on your system (It should theoretically work with Indigo as well, but it has only been tested with Hydro).

Configuration for this setup process is in ```video_device.txt```. It is currently configured to read from your computer's webcam (```/dev/video0```). If you wish to read from a different device, please edit ```video_device.txt```.

## [ar_pose](http://wiki.ros.org/ar_pose)

First we download the package that handles tracking multiple fiducials. The underlying implementation is based on OpenCV.
```bash
./ar_pose_setup.sh
```

## [gscam](http://wiki.ros.org/gscam)
We have not included install scripts for gscam. See the official ROS [documentation](http://wiki.ros.org/gscam) for more details on how install gscam. When you run ```run_gscam.sh```, gscam will capture video from the device detailed in ```video_device.txt``` and publish it to the ROS topic ```/camera/image_raw```. Keep this running for camera calibration.
```bash
./run_gscam.sh
```

## [camera_calibration](http://wiki.ros.org/camera_calibration) 

**You will need to have ```roscore``` running during this step.**

**You will need to have ```gscam``` running during this step in a separate thread. See the previous section.**

When you run ```camera_calibration.sh```, a screen should pop up showing video feed from your webcam. For instructions on how to calibrate your camera, please see the ROS [tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). This will create camera calibration yaml files that various packages use to process video.
```bash
./camera_calibration.sh
```

## [uvc_camera](http://wiki.ros.org/uvc_camera)
This is required by AR Pose. The ```ucv_camera_setup.sh``` script installs uvc_camera and copies the ```camera_calibration.yaml``` file created in the previous step to the location that uvc_camera expects.
```bash
./uvc_camera_setup.sh
```

## The Home Stretch

**You will need to have ```roscore``` running during this step.**

The last thing to do is ensure that no processes are using the video input you want to track, create a launch file that uses all of the camera calibration files you've made, and launch the application. This is all done in ```fiducial_tracking.sh```.
```bash
./fiducial_tracking.sh
```
