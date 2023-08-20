cmake_minimum_required(VERSION 2.8.3)
project(apollo2ros)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread -I/usr/local/include -L/usr/local/lib -lprotobuf")

find_package(Eigen3 REQUIRED)
#find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

catkin_package(
 LIBRARIES src
 #  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    /usr/local/include
)

file(GLOB PROTO_SRC "proto/*.cc")

add_executable( gridmap 
    gridmap.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( gridmap
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )
    
add_executable( pointcloud 
    pointcloud.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( pointcloud
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( lidar 
    lidar.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( lidar
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( tf 
    tf.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( tf
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( odometry 
    odometry.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( odometry
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( kino 
    kino.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( kino
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( minco 
    minco.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( minco
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( kino_marker 
    kino_marker.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( kino_marker
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( minco_marker 
    minco_marker.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( minco_marker
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )

add_executable( odometry_marker 
    odometry_marker.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( odometry_marker
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )


add_executable( tf_marker 
    tf_marker.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( tf_marker
    ${catkin_LIBRARIES}
    # /usr/local/lib
    protobuf
    pthread
    )