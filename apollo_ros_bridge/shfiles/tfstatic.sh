#!/bin/bash
#rosrun tf static_transform_publisher 0 0 1.7 0 0 -0.7071 0.7071 agent_0 lidar 0.01;
rosrun tf static_transform_publisher 0 0 1.7 0 0 0 1 map lidar 0.01;
