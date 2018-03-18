cmake_minimum_required(VERSION 2.8.3)
project(cpg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Fin_servos.msg
  Servo.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
set (CMAKE_CXX_STANDARD 11)
add_executable(cpg_test src/cpg_test.cpp include/CPG.cpp include/Joint.cpp)
target_link_libraries(cpg_test ${catkin_LIBRARIES})
add_dependencies(cpg_test cpg_generate_messages_cpp)

