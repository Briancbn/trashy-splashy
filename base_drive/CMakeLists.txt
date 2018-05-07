cmake_minimum_required(VERSION 2.8.3)
project(base_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Base_drive.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(twist_to_drive src/twist_to_drive.cpp)
#target_link_libraries(twist_to_drive ${catkin_LIBRARIES})
#add_dependencies(twist_to_drive base_drive_generate_messages_cpp)

add_executable(joy_drive src/joy_drive.cpp)
target_link_libraries(joy_drive ${catkin_LIBRARIES})
add_dependencies(joy_drive base_drive_generate_messages_cpp)

