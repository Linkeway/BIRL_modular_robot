# BIRL modular robot
**Note**: This modular robotic system is developed in BIRL lab and is introded in this [paper](http://ieeexplore.ieee.org/document/5354051/).
### Pre-requisites:
The **pre-requisite.sh** files will tell you what you need to install before using this repo.
### This repo contains :
   - **birl_module_canopen** : Package for canopen communication with joint modules
   - **mr_description**	     : URDF files
   - **mr_setup_detect**     : Use [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) to detect setup of a robotic systecm buit with our modules and automatically generate urdf file
   - **manipulator5d_moveit_config**: Created by moveit_setup_assistant

### Quick demo:
```
$ roslaunch mr_description manipulator5d_display.launch
```
or
```
$ roslauch mr_description climbot5d_display.launch
```
***
To communicate with the real robot(e.g. a 5 Dof modular manipulator), try
```
$ rosrun birl_module_canopen can_prepare.sh # load CAN kernal driver and start monitoring CAN bus
$ roslaunch modular_robot_control manipulator5d_control_gui.launch
$ rosservice call /driver/init
```
This shold start a position control GUI.
***
To use Moveit! to do motion planning for the robot to excute:
```bash
$ roslaunch manipulator5d_moveit_config manipulator5d_moveit.launch
$ rosservice call /driver/init
```


