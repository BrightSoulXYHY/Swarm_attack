#! /bin/bash

# cd /mnt/f/Progroming/RflySim/Swarm_ws/
# . devel/setup.bash

# start mavros to connect mav
roslaunch main 4_drone_bs.launch & PID0=$!
# roslaunch main 2_drone.launch & PID0=$!
sleep 20s

# start MatabNode
../../MatabNode/rflysimindoorcontroller_r2018b_n12_v9_node & PID4=$!
# src/MatabNode/rflysimindoorcontroller_r2018b_n12_v9_node & PID4=$!

roslaunch decision 4_drone.launch & PID1=$!
# roslaunch decision 2_drone.launch & PID1=$!
sleep 5s

# roslaunch visualization multi_visual_12.launch & PID2=$!
# roslaunch visualization multi_visual_2.launch & PID2=$!
# sleep 5s
# roslaunch attack multi_attack.launch & PID3=$!
# sleep 5s

wait
kill -9 PID0 PID1 PID2 PID3 PID4
exit
