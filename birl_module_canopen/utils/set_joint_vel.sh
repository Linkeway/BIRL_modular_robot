#!/bin/bash


if [ $# -ne 2 ]
then 
	echo "Usage:  set_joint_vel.sh  <joint_id> <velocity pulses/sec>"

	exit 0
fi


transmission_ratio=1

vel=$2
#calculate the COB-id of the SDO for the given node-id
SDO=$(( `echo "ibase=16;600"|bc` + $1 ))
COBid=`echo "obase=16;$SDO"|bc`

vel_in_cnts=`echo  "$vel*$transmission_ratio"|bc`
vel_hex=`echo "obase=16;$vel_in_cnts"|bc`

zero_8digits=00000000
num_of_0=`expr 8 - ${#vel_hex}`
vel_hex_8digits=${zero_8digits:0:${num_of_0}}${vel_hex}

msg=`echo 23.81.60.00.${vel_hex_8digits:6:2}.${vel_hex_8digits:4:2}.${vel_hex_8digits:2:2}.${vel_hex_8digits:0:2}`


cansend can0 $COBid#$msg


