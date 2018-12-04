cmake_minimum_required(VERSION 2.8.3)
project(compliance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
  rosparam_shortcuts
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES compliant_control
  CATKIN_DEPENDS geometry_msgs moveit_ros_planning_interface roscpp rosparam_shortcuts rospy sensor_msgs  std_msgs std_srvs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(compliant_control
  src/${PROJECT_NAME}/compliant_control.cpp
)

add_executable(compliant_command_node src/${PROJECT_NAME}/compliant_command_node.cpp)
add_dependencies(compliant_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(compliant_command_node ${catkin_LIBRARIES} compliant_control ${Eigen_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS compliant_control compliant_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
