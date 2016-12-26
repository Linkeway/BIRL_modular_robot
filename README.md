# BIRL modular robot
### This repo contains :
   - **birl_module_canopen** : package for canopen communication with joint modules
   - **mr_description**	     : URDF files
**Note**: The robotic system is developed in BIRL lab and is introded in this [paper](http://ieeexplore.ieee.org/document/5354051/).
### Quick demo:
```
$ roslaunch mr_description manipulator_display.launch
```
To communicate with the real robot(e.g. a 5 Dof modular manipulator), try
```
$ rosrun birl_module_canopen can_prepare.sh
$ roslaunch modular_robot_control manipulator5d_control_gui.launch
$ rosservice call /driver/init
```
This shold start a position control GUI.


