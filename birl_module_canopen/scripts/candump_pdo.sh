#!/bin/bash


if [ $# -ne 2 ]
then 
	echo "Usage:  minitor_pdo.sh <can_interface> <node ID>"
	echo "E.g. :  monitor_pdo.sh can0 1"
	exit 0
fi

echo "Candumping CAN messages with COB-id :"
echo "18$2 20$2 28$2 30$2 38$2 40$2 48$2 50$2"
echo "---------------------------------------"
pdo_cob_id=`echo "18$2|20$2|28$2|30$2|38$2|40$2|48$2|50$2"`
candump -cdex "$1" |egrep $pdo_cob_id  # candump_pdo 

