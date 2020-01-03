#!/bin/bash
echo "123qweasd111" | sudo -S /usr/local/sbin/irqbalance --debug &
sleep 120
CPUs=$(grep -c processor /proc/cpuinfo)

PIDs=$(ps aux | grep -E "nodelet|python|monitor" | awk '{print $2}')
for PID in $PIDs; do
   taskset -pc 1-7 $PID

done
