#!/bin/bash

if [ "$#" -ne 3 ]
then
	echo "Three arguments needed: nodeID, position_in_pulse, velocity_in_pulse_per_sec!"
	echo "e.g. rel_pos_ctrl_test.sh 10000 10000"
else
	zero_8digits=00000000

# pre-process position argument for cansed
	#deal with negtive position command
	if [ $2 -lt 0 ]
	then
	 pos=`expr 4294967296 + $2`
	else
	 pos=$2
	fi	
	
	# turn decimal number to hex 
	pos_in_hex=`echo "obase=16;$pos"|bc`
	#echo "pos_in_hex: ${pos_in_hex}"
	
	# complete the hex number to 8 digits
	num_of_0=`expr 8 - ${#pos_in_hex}`
	pos_in_hex_8digits=${zero_8digits:0:${num_of_0}}${pos_in_hex}
	#echo "pos_in_hex_8digits: ${pos_in_hex_8digits}"
	
	# turn the number in little-endian to big-endian
	p_hex_big_end=${pos_in_hex_8digits:6:2}${pos_in_hex_8digits:4:2}${pos_in_hex_8digits:2:2}${pos_in_hex_8digits:0:2}
	#echo "pos_hex_big_end: ${p_hex_big_end}"
	
# pre-process velocity argument for cansed
	#deal with negtive velocity command
	if [ $3 -lt 0 ]
	then
	    vel=`expr 0 - $3`
	else
	    vel=$3
	fi
	#turn decimal number to hex
	vel_in_hex=`echo "obase=16;$vel"|bc`

	
	#complete the hex number to 8 digits
	num_of_0=`expr 8 - ${#vel_in_hex}`
	vel_in_hex_8digits=${zero_8digits:0:${num_of_0}}${vel_in_hex}
	
	
	#turn the number in little-endian to big-endian
	v_hex_big_end=${vel_in_hex_8digits:6:2}${vel_in_hex_8digits:4:2}${vel_in_hex_8digits:2:2}${vel_in_hex_8digits:0:2}
	
	#COB id 0x600+id
	cob_id=`expr 1536 + $1` 
	COB_id=`echo "obase=16;$cob_id"|bc`
	
	# configuring the PDOs
	cansend can0 ${COB_id}#2f.00.16.00.00.00.00.00 # RPDO1 stop
	cansend can0 ${COB_id}#23.00.16.01.10.00.40.60 # map to control word
	cansend can0 ${COB_id}#2f.00.16.00.01.00.00.00 # RPDO1 enable

	cansend can0 ${COB_id}#2f.01.16.00.00.00.00.00 # RPDO2 stop
	cansend can0 ${COB_id}#23.01.16.01.20.00.7a.60 # map to target position
	cansend can0 ${COB_id}#23.01.16.02.20.00.81.60 # profile velocity
	cansend can0 ${COB_id}#2f.01.16.00.02.00.00.00 # RPD0 enable

	cansend can0 000#01.02                   # start the node

	cansend can0 ${COB_id}#2f.60.60.00.01.00.00.00 # control mode Profile Position
	cansend can0 ${COB_id}#2b.40.60.00.06.00.00.00 # set control word 6
	cansend can0 ${COB_id}#2b.40.60.00.07.00.00.00 # set 7
	cansend can0 ${COB_id}#2b.40.60.00.0f.00.00.00 # set F, enable motor

	#COB id 0x300+id
	cob_id=`expr 768 + $1` 
	COB_id=`echo "obase=16;$cob_id"|bc`
	cansend can0 ${COB_id}#${p_hex_big_end}${v_hex_big_end} # send command

	#COB id 0x200+id
	cob_id=`expr 512 + $1` 
	COB_id=`echo "obase=16;$cob_id"|bc`
	
	#clear the control word for next move
	cansend can0 ${COB_id}#0f00                    # clear bit4
	cansend can0 ${COB_id}#5f00                    # set bit6 and bit4
	cansend can0 ${COB_id}#0f00                    # clear bit4

	#start the motor
	cansend can0 ${COB_id}#0f00                    # clear 6040 bit4
	cansend can0 ${COB_id}#1f00       # set bit6: incremental;set bit4:motor start

fi
