#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./ctrl1_rt
else
	EXEC=./ctrl1_nrt
fi

$EXEC -n x4_0 -a 127.0.0.1 -p 9000 -l /tmp -x setup_x4.xml -t x4_simu 
