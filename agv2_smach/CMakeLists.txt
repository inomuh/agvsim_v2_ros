cmake_minimum_required(VERSION 2.8.3)
project(agv2_smach)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(FILES TaskService.srv)


generate_messages(DEPENDENCIES std_msgs)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
