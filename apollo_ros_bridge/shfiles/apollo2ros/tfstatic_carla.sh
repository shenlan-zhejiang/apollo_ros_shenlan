#!/bin/bash

echo "child_frame_id: lidar
transform:
  x: 0 
  y: 0 
  z: 2.4
rotation: 
  x: 0 
  y: 0 
  z: 0
  w: 1"
  
rosrun tf static_transform_publisher 0 0 2.4 0 0 0 1 agent_0 lidar 0.01;
