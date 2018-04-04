cmake_minimum_required(VERSION 2.8.3)
project(act_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  rdr_ugv_control
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Act_drive.msg
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

add_executable(twist_to_drive src/twist_to_drive.cpp)
target_link_libraries(twist_to_drive ${catkin_LIBRARIES})
add_dependencies(twist_to_drive act_drive_generate_messages_cpp)

add_executable(ugv_to_drive src/ugv_to_drive.cpp)
target_link_libraries(ugv_to_drive ${catkin_LIBRARIES})
add_dependencies(ugv_to_drive act_drive_generate_messages_cpp)


add_executable(act_joy_drive src/act_joy_drive.cpp)
target_link_libraries(act_joy_drive ${catkin_LIBRARIES})
add_dependencies(act_joy_drive act_drive_generate_messages_cpp)

