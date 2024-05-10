cmake_minimum_required(VERSION 3.0.2)
project(merakm_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES merakm_bringup
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_odom src/robot_odom.cpp)
target_link_libraries(robot_odom ${catkin_LIBRARIES})

add_executable(robot_cmd src/robot_cmd.cpp)
target_link_libraries(robot_cmd ${catkin_LIBRARIES})