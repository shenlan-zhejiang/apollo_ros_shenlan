#!/bin/bash
cd ~/apollo_ros_bridge/apollo2ros_kino;
source devel/setup.bash;
rosparam set use_sim_time false;
rosrun kino kino;
