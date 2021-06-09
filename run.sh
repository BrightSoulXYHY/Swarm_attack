#!/bin/bash

# cd ~/Swarm_ws
. devel/setup.bash

# if [ ! -f "devel/lib/decision/rflysimindoorcontroller_r2018b_n12_v9_node" ];
# then

#     echo "no rflysimindoorcontroller_r2018b_n12_v9_node"
#     if [ ! -d "devel/lib/decision" ];
#     then
#         mkdir devel/lib/decision
#     fi
#     chmod +x src/MatabNode/rflysimindoorcontroller_r2018b_n12_v9_node
#     cp src/MatabNode/rflysimindoorcontroller_r2018b_n12_v9_node devel/lib/decision/

# fi


let MAVID=${HOSTNAME:4:2}
echo "rflysimindoorcontroller_r2018b_n12_v9_node ready!"
echo "this MAV id :${MAVID}"


roslaunch rflysim_ros_pkg cameras.launch   & PID3=$!
sleep 5s

roslaunch bs_assis bs_dds.launch  mav_id:=${MAVID}  & PID0=$!
sleep 5s


roslaunch decision multi_drone_bs.launch  drone_id:=${MAVID}  & PID1=$!
sleep 5s

#roslaunch visualization multi_visual_bs.launch  drone_id:=${MAVID} type:=207 & PID2=$!

wait
kill -9 PID0 PID1 PID2 PID3
exit
