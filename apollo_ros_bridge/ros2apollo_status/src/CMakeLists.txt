cmake_minimum_required(VERSION 2.8.3)
project(status)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -L/usr/local/lib -lprotobuf")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
  message_generation
  geometry_msgs
  carla_msgs
)

find_package(Eigen3 REQUIRED)
# find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs carla_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include
    /usr/local/include
    /usr/include/eigen3
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    include/mpc
)

file(GLOB PROTO_SRC "proto/*.cc")

add_executable( status 
    status.cpp
    ${PROTO_SRC}
    bridge/bridge_header.cc
    include/common/carla_adapter.h)

target_link_libraries( status
    ${catkin_LIBRARIES}
    /usr/local/lib
    protobuf
    pthread
    )