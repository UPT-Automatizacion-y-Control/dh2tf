cmake_minimum_required(VERSION 3.0.2)
project(dh2tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  DHParameters.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dh2tf src/dh2tf.cpp)
target_link_libraries(dh2tf ${catkin_LIBRARIES})
add_dependencies(dh2tf dh2tf_generate_messages_cpp)

