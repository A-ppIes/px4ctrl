cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros_msgs
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/fake_rc.cfg
 )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES px4ctrl
   CATKIN_DEPENDS dynamic_reconfigure quadrotor_msgs roscpp rospy geometry_msgs sensor_msgs mavros_msgs
   DEPENDS Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

set(LIBS
${catkin_LIBRARIES}
${EIGEN3_LIBS}
)

add_executable(px4ctrl_node 
  src/PX4Ctrlnode.cpp
  src/PX4Ctrlfsm.cpp
  src/PX4Ctrlparam.cpp
  src/controller.cpp
  src/ROScallback.cpp
)


target_link_libraries(px4ctrl_node
  ${LIBS}
)

#catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
