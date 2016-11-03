#!/bin/bash




radian2degree(){
while read data; do
	echo "57.3*${data}"|bc 
done
}
if [ $# -ne 1]
then 
	echo "This script is for converting the position angle in topic from radian to degree."
	echo "Usage: topic_pos_radian2degree.sh <topic>!"
	exit 0
fi
rostopic  echo $1|grep pos|tr -d "position: ["|tr -d ]| radian2degree

