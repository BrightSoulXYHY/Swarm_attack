#! /bin/bash

# Set FC into Hex+ mode
roslaunch main 12_drone.launch & PID0=$!
sleep 20s
# ~/Swarm_ws/devel/lib/rflysimindoorcontroller_r2018b_n12_v9/rflysimindoorcontroller_r2018b_n12_v9_node & PID4=$!
# Swarm_ws/src/MatabNode/rflysimindoorcontroller_r2018b_n12_v9_node & PID4=$!
roslaunch decision 12_drone.launch & PID1=$!
sleep 5s
roslaunch visualization multi_visual_12.launch & PID2=$!
sleep 5s
roslaunch attack multi_attack.launch & PID3=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2 PID3 PID4
exit
