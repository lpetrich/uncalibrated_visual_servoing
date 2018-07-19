cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

project(uncalibrated_visual_servoing)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  message_generation
  cmake_modules
  wam_control
)

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  TaskIds.msg
  TrackPoint.msg
  TrackedPoints.msg
  Error.msg
  ErrorInfo.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES uncalibrated_visual_servoing
  CATKIN_DEPENDS message_runtime
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(uvs src/uvs_control.cpp)
target_link_libraries(uvs
  ${catkin_LIBRARIES}
  arm_control
  bhand_control
)

add_executable(error_calculation src/error_calculation.cpp)

target_link_libraries(error_calculation
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
 )
