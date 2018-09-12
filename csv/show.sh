#/bin/bash

function msleep()
{
	xte "keydown Alt_L"
	# xte "usleep 10000"
	xte "usleep 300000"
	# xte "usleep 150000"
	xte "key f"
	xte "usleep 10000"
	# xte "usleep 150000"
	xte "keyup Alt_L"
	# xte "sleep 0.1"
	# xte "usleep 150000"
	xte "key l"
	sleep $1
}

duration=1

if [ $# -eq 2 ]; then
	duration=$2
elif [ $# -ne 1 ]; then
	echo set human id
	echo -e "e.g. $PWD$ ./show.sh 5"
	exit
fi

human_id=$1

echo click Excel sheet
sleep 3s

cnt=0; for i in `ls -v gt/${human_id}`; do ./setValue.sh ${human_id} ${i%.csv} && echo $i \($((++cnt))\) && echo -e "click Excel sheet\n\n" && msleep ${duration}s; done


