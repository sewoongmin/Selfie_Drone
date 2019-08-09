cmake_minimum_required(VERSION 2.8.3)
project(selfie_drone)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MsgState.msg
)

# add_service_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_tracer
  CATKIN_DEPENDS rospy std_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(selfie_pid src/selfie_pid.cpp)
add_dependencies(selfie_pid ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(selfie_pid ${catkin_LIBRARIES})