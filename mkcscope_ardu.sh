#!/bin/sh   

#SDIR=/home/jhkang/Project/33_ardupilot_latest
SDIR=$PWD

rm -rf cscope.out cscope.files   
#find . \( -name '*.c' -o -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.s' -o -name '*.S' -o -name '*.java' \) -type f -print > cscope.files   
find $SDIR \
       -path "$SDIR/AntennaTracker/*" -prune -o \
	   -path "$SDIR/APMrover2/*"  -prune -o \
	   -path "$SDIR/ArduPlane/*" -prune -o \
	   -path "$SDIR/ArduSub/*" -prune -o \
	   -path "$SDIR/benchmarks/*" -prune -o \
	   -path "$SDIR/tests/*" -prune -o \
	   -path "$SDIR/libraries/AP_HAL_Empty/*" -prune -o \
	   -path "$SDIR/libraries/AP_HAL_SITL/*" -prune -o \
	   -path "$SDIR/libraries/AP_HAL_Linux/*" -prune -o \
	   -path "$SDIR/*/examples/*" -prune -o \
	   \( -name '*.c' -o -name '*.cpp' -o -name '*.cc' -o -name '*.h' -o -name '*.s' -o -name '*.S' -o -name '*.java' \) -type f -print > cscope.files   
cscope -I cscope.files  

#출처: http://huskey27.tistory.com/entry/CSCOPE-사용 [huskey27]
