cmake_minimum_required(VERSION 2.8.3)
project(uav_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Delta.msg
  Command.msg
  State.msg
  Status.msg
  Sensors.msg
  Waypoint.msg
  WaypointArray.msg
  Path.msg
)

#add_service_files(
#  DIRECTORY srv
#  FILES
#  ParamFile.srv
#  ParamGet.srv
#  ParamSet.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs
)
