cmake_minimum_required(VERSION 2.8.3)
project(hiro_panda_ik)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  sensor_msgs
  std_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  tf
  trac_ik_lib
  franka_control
  franka_gripper
  pluginlib
  realtime_tools
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES hiro_panda_ik_controllers
 CATKIN_DEPENDS 
  controller_interface 
  franka_control 
  hardware_interface 
  franka_hw 
  message_runtime
  pluginlib
  realtime_tools
  roscpp
  geometry_msgs  
 DEPENDS Franka
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(hiro_panda_ik_controllers
  src/panda_utils/panda_trac_ik.cpp
  src/panda_interface/panda_pose_controller.cpp
)

target_link_libraries(hiro_panda_ik_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS hiro_panda_ik_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES panda_ik_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)