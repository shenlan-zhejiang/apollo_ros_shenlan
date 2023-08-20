cmake_minimum_required(VERSION 2.8.3)
project(mpc)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
  message_generation
  carla_msgs
)
find_package(Eigen3 REQUIRED)
# find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  SingleMinco.msg
  MincoTraj.msg
  Trajectory.msg
  )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs carla_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
)

add_library(mpc
  src/mpc.cpp
  src/cubic_spline_planner.cpp
        include/common/carla_adapter.h)

add_dependencies(mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc
    # PRIVATE osqp::osqp
    OsqpEigen::OsqpEigen
    ${catkin_LIBRARIES}
    )

add_executable(mpc_controller_node
    src/mpc_node.cpp
  )
target_link_libraries(mpc_controller_node
  mpc
  ${catkin_LIBRARIES}
)
