#!/bin/bash
cd ~/apollo_ros_bridge/apollo2ros_tf;
source devel/setup.bash;
rosparam set use_sim_time false;
rosrun tf tf;
