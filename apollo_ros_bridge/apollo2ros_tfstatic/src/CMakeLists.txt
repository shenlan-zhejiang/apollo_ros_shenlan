cmake_minimum_required(VERSION 2.8.3)
project(tfstatic)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread -I/usr/local/include -L/usr/local/lib -lprotobuf")

#find_package(Eigen3 REQUIRED)
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
    /usr/local/include
)

file(GLOB PROTO_SRC "proto/*.cc")

add_executable( tfstatic 
    tfstatic.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    )

target_link_libraries( tfstatic
    ${catkin_LIBRARIES}
    /usr/local/lib
    protobuf
    pthread
    )  
