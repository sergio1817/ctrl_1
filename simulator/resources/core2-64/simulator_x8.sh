#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./ctrl1_simulator_rt
else
	EXEC=./ctrl1_simulator_nrt
fi

$EXEC -n x8_0 -t x8 -p 9000 -a 127.0.0.1 -x simulator_x8.xml -o 10 -m $FLAIR_ROOT/flair-src/models -s $FLAIR_ROOT/flair-src/models/indoor_flight_arena.xml
