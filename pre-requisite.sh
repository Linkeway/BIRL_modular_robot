

sudo apt-get install ros-indigo-ros-control ros-indigo-ros-canopen
sudo pip install python-can
cat "[default]
interface = socketcan
channel = can0" >> /etc/can.conf
# sudo apt-get install ros-indigo-realtime-tools ros-indigo-control-toolbox libmuparser2;
# git clone https://github.com/ros-industrial/ros_canopen.git;
# cd ros_canopen/;git checkout indigo-devel;
# cd ..;git clone https://github.com/ros-controls/ros_control.git;
# cd ros_control/;git checkout indigo-devel;

cd src/;
git clone https://github.com/Linkeway/BIRL_modular_robot.git
git clone https://github.com/Linkeway/robot_model.git
cd robot_model;
git checkout indigo-devel


cd;mkdir 3rd_party_lib;cd 3rd_party_lib;git clone https://github.com/beltoforion/muparser.git;cd muparser/;./configure;make;sudo make install;

#32 bit ECI:
#63 bit ECI:
#cd ~/3rd_party_lib/EciLinux_amd64/bin/release; ./LinuxEciDemo;#if can't find devices,run the below script
# ./LinuxEciDeviceAdmin -h;
#cd ~/3rd_party_lib/EciLinux_amd64/src/EciPythonDemos;python PythonEciDemo.py

#rm 60-eci.rules /etc/udev/rules.d

#socketCAN Driver for Linuxe
cd  3rd_party_lib/
mkdir socketcan_driver
cd socketcan_driver
wget http://www.ixxat.com/docs/librariesprovider8/default-document-library/downloads/other-drivers/socketcan_1-1-92-0_20150508.zip?sfvrsn=10
unzip socketcan_1-1-92-0_20150508.zip?sfvrsn=10
gedit README &
cd usb-to-can_v2_socketcan
#then follow the instructions in README!!!!  NOTE: try "lsmod | grep can_dev" if "lsmod | grep can-dev" doesn't work
#when Ubuntu system kernal updated, then the can-dev module might need to be re-installed .
#to monitor the can bus data , use " candump -dex can0 ", to send data to CAN bus use " cansend can0 123#112233 "
