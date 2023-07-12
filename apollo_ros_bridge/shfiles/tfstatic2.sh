#!/bin/bash
cd ~/apollo_ros_bridge/apollo2ros_tfstatic2;
source devel/setup.bash;
rosparam set use_sim_time false;
rosrun tfstatic2 tfstatic2;
