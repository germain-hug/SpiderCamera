cmake_minimum_required(VERSION 2.8.3)
project(cmd_vel_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   cmd_vel_motors.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs bbox_to_cmd_vel)
include_directories(include ${catkin_INCLUDE_DIRS} /home/pi/catkin_ws/src/cmd_vel_controller/include/avr /home/pi/catkin_ws/src/cmd_vel_controller/include/Bioloid)

add_executable(ax12_controller src/ax12_controller.cpp)
target_link_libraries(ax12_controller ${catkin_LIBRARIES})
add_dependencies(ax12_controller cmd_vel_controller_generate_messages_cpp)

