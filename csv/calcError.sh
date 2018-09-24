#!/bin/bash

sum_cnt()
{
	while read -d $'\0' dir; do
		if [ -e ./pr/${dir#./gt/} ]; then # pr内にもdir(数字)あったら
			for i in `ls ${dir}`;do # gt/dir(数字)/ 内全部csv見る
				if [ -e ./pr/${dir#./gt/}/$i ]; then # pr内にもcsvあったら
					paste ${dir}/$i ./pr/${dir#./gt/}/$i |\
						awk -F'[ ,\t]' '{sum+=sqrt(($1-$3)^2 + ($2-$4)^2)} END{print sum" "NR}'
				fi
			done
		fi
	done < <(find ./gt -mindepth 1 -maxdepth 1 -print0) # gt内のdir(数字)全部みる
}

sum_cnt | awk '{sum+=$1}{cnt+=$2} END{print sum/cnt" "cnt}'

