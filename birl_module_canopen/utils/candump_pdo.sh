#!/bin/bash


if [ $# -ne 2 ]
then 
	echo "Usage:  minitor_pdo.sh <can_interface> <node ID>"
	echo "E.g. :  monitor_pdo.sh can0 1"
	exit 0
fi

#calculate the COB-id for the given node-id
tpdo1=$(( `echo "ibase=16;180"|bc` + $2 ))
tpdo1=`echo "obase=16;$tpdo1"|bc`

tpdo2=$(( `echo "ibase=16;280"|bc` + $2 ))
tpdo2=`echo "obase=16;$tpdo2"|bc`

tpdo3=$(( `echo "ibase=16;380"|bc` + $2 ))
tpdo3=`echo "obase=16;$tpdo3"|bc`

tpdo4=$(( `echo "ibase=16;480"|bc` + $2 ))
tpdo4=`echo "obase=16;$tpdo4"|bc`


rpdo1=$(( `echo "ibase=16;200"|bc` + $2 ))
rpdo1=`echo "obase=16;$rpdo1"|bc`

rpdo2=$(( `echo "ibase=16;300"|bc` + $2 ))
rpdo2=`echo "obase=16;$rpdo2"|bc`

rpdo3=$(( `echo "ibase=16;400"|bc` + $2 ))
rpdo3=`echo "obase=16;$rpdo3"|bc`

rpdo4=$(( `echo "ibase=16;500"|bc` + $2 ))
rpdo4=`echo "obase=16;$rpdo4"|bc`

echo "Candumping CAN messages with COB-id :"
echo "---------------------------------------"
echo "tdpo1-$tpdo1 tdpo2-$tpdo2 tdpo3-$tpdo3 tdpo4-$tpdo4"
echo "rdpo1-$rpdo1 rdpo2-$rpdo2 rdpo3-$rpdo3 rdpo4-$rpdo4"
echo "---------------------------------------"
pdo_cob_id=`echo "18$2|20$2|28$2|30$2|38$2|40$2|48$2|50$2"`
candump -cdex "$1" |egrep $pdo_cob_id  # candump_pdo 

