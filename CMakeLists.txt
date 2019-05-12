cmake_minimum_required(VERSION 2.8.3)
project(control_by_web_x4xx_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#catkin_python_setup()

add_message_files(
  FILES
  X4xx_status.msg  
  X4xx_relay_current_status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

