#!/usr/bin/env bash



for filename in $(pwd)/*; do
	[ -e "$filename" ] || continue

	extension="${filename##*.}"
	filename=$(basename $filename)
	noextension="${filename%.*}"
	
	if [ $extension = bag ]; then
		echo $filename
		echo $extension
		echo $noextension

		rostopic echo -b $filename -p /landing/arduino_pos > $noextension"_raw.csv"
		rostopic echo -b $filename -p /landing/kalman_pos > $noextension"_kalman.csv"
	fi

done
