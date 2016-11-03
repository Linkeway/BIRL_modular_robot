#!/bin/bash
# unfinished
# function for interpreting PDO data frames. The PDO settings can be seen at page 25 http://www.copleycontrols.com/Motion/pdf/CANopenProgrammersManual.pdf .

pdo_calc(){

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
	
	pdos="$tpdo1|$tpdo2|$tpdo3|$tpdo4|$rpdo1|$rpdo2|$rpdo3|$rpdo4"
	
	while read data; do 
		#printf "%s" "${data:30:2}"
		#printf "\n"
		pdo_msg=`egrep $pdos <<< ${data} `
		if test -n "$pdo_msg"
		then
			num_zero=`expr 7 - ${pdo_msg:29:1}`
			#number=`grep -o '\`' <<< $pdo |wc -l`
			#num_zero=`expr 7 - $number`
			echo $num_zero
			
			value=`echo "ibase=16;${pdo_msg:33:2}${pdo_msg:36:2}${pdo_msg:39:2}${pdo_msg:42:2}${pdo_msg:45:2}${pdo_msg:48:2}${pdo_msg:51:2}${pdo_msg:54:2}" |bc`
			printf "%s: " "${pdo_msg:0:29}"
			printf "%s\n" "${value}"
		fi
	done
}



if [ $# -ne 1 ]
then
	echo "Usage: candump_calc_pdo <caninterface> <node-id>"
	echo "example: candump_calc_pdo can0 1"
	echo "unfinished utility!!!!"
	exit 0
fi 
candump -cdexS $1 |pdo_calc 

