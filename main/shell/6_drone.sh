#! /bin/bash

# Set FC into Hex+ mode
roslaunch main 6_drone.launch & PID0=$!
sleep 30s
roslaunch decision 6_drone.launch & PID1=$!
sleep 5s
roslaunch visualization multi_visual_launch.launch & PID2=$!
sleep 5s
roslaunch attack multi_attack.launch & PID3=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2 PID3
exit