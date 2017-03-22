**This package contains codes for commanding modular robots.**
***
- Joint space control\
E.g. ```roslaunch moduar_robot_control manipulator5D_control_gui.launch```

- Cartisian space control using interactive marker to specify end-effector pose\
E.g. ```roslaunch modular_robot_control manipulator5D_cartesian_control.launch ``` 

- Motion commanding by interpreting offline generated MRL (Modular Robot Language) files\
E.g. ```roslaunch modular_robot_control manipulator5D_offline.launch [mrl_file:=...] [sim:=true]```