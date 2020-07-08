#!/bin/bash
echo "123" | sudo -s &
sleep 1
CPUs=$(grep -c processor /proc/cpuinfo)
MAX=`expr $CPUs - 1`
echo "max:"
echo $MAX
PIDs=$(ps aux | grep -E "nodelet|python|monitor" | awk '{print $2}')
for PID in $PIDs; do
   taskset -pc 1-$MAX $PID

done
