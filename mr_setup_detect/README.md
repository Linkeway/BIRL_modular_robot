## mr_setup_detect
### Descritpion
This package uses [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) to **detect the setup of a robotic system** built with our modules. Each module is attached with a tag. The tag ID indicates the serial number of the module.
### Pre-requisites
1. A Camera and its ROS driver(e.g. [Kinect](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration), Xtion, Primesense or a USB webcam).
2. ``` sudo apt-get install ros-indigo-ar_track_alvar```
### Usage

1. Usage with *launch* file:
```bash
roslaunch mr_setup_detec robot_setup_detect.launch
```
You can speicify a name for the robot:
```bash
roslaunch mr_setup_detect robot_setup_detect.launch robot_name:=YOUR_ROBOT_NAME
```
This will open Kinect2 and uses its rgb data for ar_marker detection.
2. You can also use other cameras to produce image data needed for marker detection, and start the **setup_detector** node, which subcribe to **/ar_pose_marker** and automatically generate **YOUR_ROBOT_NAME.urdf.xacro**, **YOUR_ROBOT_NAME_display.launch** and **YOUR_ROBOT_NAME.rviz** to [mr_description](https://github.com/Linkeway/BIRL_modular_robot/tree/master/mr_description) package.
```bash
rosrun mr_setup_detect setup_detector.py
``` 
You can speicify a name for the robot:
```bash
rosrun mr_setup_detect setup_detector.py YOUR_ROBOT_NAME
``` 
