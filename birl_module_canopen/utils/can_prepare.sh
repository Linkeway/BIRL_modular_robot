#!/bin/bash

device=${1-can0}
shift

bitrate=${1-1000000}
shift 

sudo modprobe can-dev # or sudo modprobe can_dev 
sudo ip link set $device down
sudo ip link set $device up type can bitrate $bitrate 
candump -dcex $device  # |egrep -v "080|77f"
