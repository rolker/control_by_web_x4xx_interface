cmake_minimum_required(VERSION 2.8.3)
project(cbw_X4xx_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  X4xx_status.msg
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS message_runtime)

