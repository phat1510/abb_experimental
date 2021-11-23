cmake_minimum_required(VERSION 3.0.2)
project(cr_msg)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Point.msg
  Type.msg
  Pose.msg
  PoseArray.msg
  Quaternion.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
