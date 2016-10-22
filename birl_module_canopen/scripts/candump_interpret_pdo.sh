#!/bin/bash


# unfinished
pdo_calc(){
	while read data; do 
		#printf "%s" "${data:30:2}"
		#printf "\n"
		pdo=`egrep "182|202|282|302|382|402|482|502" <<< ${data} `
		if test -n "$pdo"
		then
			number=`grep -o '\`' <<< $pdo |wc -l`
			num_zero=`expr 7 - $number`
			echo $num_zero
			
			value=`echo "ibase=16;${pdo:33:2}${pdo:36:2}${pdo:39:2}${pdo:42:2}${pdo:45:2}${pdo:48:2}${pdo:51:2}${pdo:54:2}" |bc`
			printf "%s: " "${pdo:0:29}"
			printf "%s\n" "${value}"
		fi
	done
}



if [ $# -ne 1 ]
then
	echo "Usage: candump_calc_pdo <caninterface>"
	echo "example: candump_calc_pdo can0."
	echo "this unfinished utility!!!!"
else
candump -cdexS $1 |pdo_calc 
fi
