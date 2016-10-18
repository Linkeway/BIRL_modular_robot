#!/bin/bash

if [ "$#" -ne 2 ]
then
	echo "Two arguments needed: position_in_pulse velocity_in_pulse_per_sec!"
else
	zero_8digits=00000000

# pre-process position argument for cansed
	# turn decimal number to hex 
	pos_in_hex=`echo "obase=16;$1"|bc`
	echo "pos_in_hex: ${pos_in_hex}"
	
	# complete the hex number to 8 digits
	num_of_0=`expr 8 - ${#pos_in_hex}`
	pos_in_hex_8digits=${zero_8digits:0:${num_of_0}}${pos_in_hex}
	#echo "pos_in_hex_8digits: ${pos_in_hex_8digits}"
	
	# turn the number in little-endian to big-endian
	p_hex_big_end=${pos_in_hex_8digits:6:2}${pos_in_hex_8digits:4:2}${pos_in_hex_8digits:2:2}${pos_in_hex_8digits:0:2}
	echo "pos_hex_big_end: ${p_hex_big_end}"
	
# pre-process velocity argument for cansed
	#turn decimal number to hex
	vel_in_hex=`echo "obase=16;$2"|bc`

	
	#complete the hex number to 8 digits
	num_of_0=`expr 8 - ${#vel_in_hex}`
	vel_in_hex_8digits=${zero_8digits:0:${num_of_0}}${vel_in_hex}
	
	
	#turn the number in little-endian to big-endian
	v_hex_big_end=${vel_in_hex_8digits:6:2}${vel_in_hex_8digits:4:2}${vel_in_hex_8digits:2:2}${vel_in_hex_8digits:0:2}
	
	
	

	

	## configuring the PDOs
	#cansend can0 602#2f.00.16.00.00.00.00.00 # RPDO1 stop
	#cansend can0 602#23.00.16.01.10.00.40.60 # map to control word
	#cansend can0 602#2f.00.16.00.01.00.00.00 # RPDO1 enable

	#cansend can0 602#2f.01.16.00.00.00.00.00 # RPDO2 stop
	#cansend can0 602#23.01.16.01.20.00.7a.60 # map to target position
	#cansend can0 602#23.01.16.02.20.00.81.60 # profile velocity
	#cansend can0 602#2f.01.16.00.02.00.00.00 # RPD0 enable

	#cansend can0 000#01.02                   # start the node

	#cansend can0 602#2f.60.60.00.01.00.00.00 # control mode Profile Position
	#cansend can0 602#2b.40.60.00.06.00.00.00 # set control word 6
	#cansend can0 602#2b.40.60.00.07.00.00.00 # set 7
	#cansend can0 602#2b.40.60.00.0f.00.00.00 # set F, enable motor
	#cansend can0 302#50.c3.c1.11.2c.01.10.00 # send command
	cansend can0 302#${p_hex_big_end}${v_hex_big_end}

	##clear the control word for next move
	#cansend can0 202#0f00                    # clear bit4
	#cansend can0 202#5f00                    # set bit6 and bit4
	#cansend can0 202#0f00                    # clear bit4

	##start the motor
	#cansend can0 202#0f00                    # clear 6040 bit4
	#cansend can0 202#1f00       # set bit6: incremental;set bit4:motor start

fi
