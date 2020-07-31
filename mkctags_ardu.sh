#!/bin/bash

rm -rf ./tags

find . \
-path "./AntennaTracker" -prune -o \
-path "./Rover" -prune -o \
-path "./ArduPlane" -prune -o \
-path "./ArduSub" -prune -o \
-path "./tests" -prune -o \
-path "./libraries/AP_HAL_Empty" -prune -o \
-path "./libraries/AP_HAL_SITL" -prune -o \
-path "./libraries/AP_HAL_Linux" -prune -o \
-path "./libraries/AP_HAL_Linux/examples" -prune -o \
-path "./*/examples" -prune -o \
-path "*.py" -prune -o \
-path "*.cxx" -prune -o \
-path "*.cc" -prune -o \
-path "*.java" -prune -o \
-print > .ctagignore

ctags -L .ctagignore

rm -rf .ctagigmore

