#/bin/bash

duration=5

if [ $# -eq 1 ]
then
	duration=$1
fi

cnt=0; for i in `ls -v gt/5`; do ./setValue.sh 5 ${i%.csv} && echo $i \($((++cnt))\) && echo -e "check Excel sheet (Alt+f, l)\n\n" && sleep ${duration}s; done


