#!/bin/bash


if [ $# -ne 3 ]
then 
	echo "Usage:  set_joint_vel.sh <joint_type> <joint_id> <velocity rad/sec>"
	echo "E.g. :  set_joint_vel.sh T 1 3.15"
	echo "        set_joint_vel.sh I 1 3.15"
	exit 0
fi

if [ $1 == T ];then
	transmission_ratio=480
elif [ $1 == I ];then
	transmission_ratio=457
else
	echo "No such Joint type! T or I or t or i supported!"
fi
vel=$2
#calculate the COB-id of the SDO for the given node-id
SDO=$(( `echo "ibase=16;600"|bc` + $1 ))
COBid=`echo "obase=16;$SDO"|bc`

vel_in_cnts=`echo  "$vel*$transmission_ratio*4096/2/3.14159"|bc`
vel_hex=`echo "obase=16;$vel_in_cnts"|bc`

zero_8digits=00000000
num_of_0=`expr 8 - ${#vel_hex}`
vel_hex_8digits=${zero_8digits:0:${num_of_0}}${vel_hex}

msg=`echo 23.81.60.00.${vel_hex_8digits:6:2}.${vel_hex_8digits:4:2}.${vel_hex_8digits:2:2}.${vel_hex_8digits:0:2}`


cansend can0 $COBid#$msg


