cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-ignored-attributes")
project(uncalibrated_visual_servoing)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
  roscpp
  std_msgs
  message_generation
  cmake_modules
  wam_control
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
)

find_package(Eigen REQUIRED)

catkin_python_setup()

#add_message_files(
#  FILES
#  TaskIds.msg
#  TrackPoint.msg
#  TrackedPoints.msg
#  EndEffectorPoints.msg
#  Error.msg
#  ErrorInfo.msg
#  Teleop.msg
#)

generate_messages(
  DEPENDENCIES
)


catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES hil_servoing
  CATKIN_DEPENDS message_runtime
  #  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#add_executable(main src/main.cpp)
#target_link_libraries(hil_servoing
#  ${catkin_LIBRARIES}
#  arm_control
#  bhand_control
#)

#add_executable(error src/calculate_error_vector.cpp)
#target_link_libraries(error
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#)

#add_executable(kinematics src/kinematics.cpp)
#target_link_libraries(kinematics
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#)

#add_executable(state_display_tutorial src/state_display_tutorial.cpp)
#target_link_libraries(state_display_tutorial
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS 
  scripts/uvs_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)