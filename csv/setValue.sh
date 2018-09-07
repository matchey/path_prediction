#/bin/bash

if [ $# -ne 2 ]
then
	echo "set time"
	exit
fi

id=$1
timestamp=$2

if [ ! -e gt/${id}/${timestamp}.csv ] || [ ! -e pr/${id}/${timestamp}.csv ]; then
	echo coudnt find file
	exit
fi

sfile="gt_pr.xlsx"

mkdir unzip_tmp

unzip $sfile -d unzip_tmp

cd unzip_tmp

# cat xl/worksheets/sheet1.xml | grep -o '<c [^<]*><v>[^<]*</v></c>' | tr '><"' '   ' | awk '{print $3,$9}'
sed -r 's/(<c [^<]*><v>)[^<]*(<\/v><\/c>)/\1\n#####\n\2/g' xl/worksheets/sheet1.xml > tmp_###.xml

x_gt=()
y_gt=()
x_pr=()
y_pr=()

FILENAME="../gt/${id}/${timestamp}.csv"
while read LINE;do
	IFS=','
	set -- $LINE
	x_gt+=($1)
	y_gt+=($2)
done < $FILENAME

FILENAME="../pr/${id}/${timestamp}.csv"
while read LINE;do
	IFS=','
	set -- $LINE
	x_pr+=($1)
	y_pr+=($2)
done < $FILENAME

count=0
cat tmp_###.xml | while read LINE || [ -n "${LINE}" ];do
	if [ "$LINE" = "#####" ]; then
		mod=$((${count}%4))
		case ${mod} in
			0 ) echo -n ${x_gt[$(((${count}-${mod}) / 4))]} ;;
			1 ) echo -n ${y_gt[$(((${count}-${mod}) / 4))]} ;;
			2 ) echo -n ${x_pr[$(((${count}-${mod}) / 4))]} ;;
			3 ) echo -n ${y_pr[$(((${count}-${mod}) / 4))]} ;;
		esac
		count=$((++count))
	else
		echo -n $LINE
	fi
done > tmp_xy.xml

mv -f tmp_xy.xml xl/worksheets/sheet1.xml

zip -r file_tmp.xlsx *

mv -f file_tmp.xlsx ../$sfile

cd ..

rm -rf unzip_tmp

