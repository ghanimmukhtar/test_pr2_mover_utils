cmake_minimum_required(VERSION 2.8.3)
project(test_pr2_mover_utils)

find_package(catkin REQUIRED COMPONENTS
  pr2_mover_utils
  roscpp
  rospy
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pr2_mover_utils roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(include /home/ghanim/git/catkin_ws/install/)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_pr2_mover_utils src/test_pr2_mover_utils.cpp)
target_link_libraries(test_pr2_mover_utils ${catkin_LIBRARIES}  )

add_executable(service_user src/service_user.cpp)
target_link_libraries(service_user ${catkin_LIBRARIES}  )

