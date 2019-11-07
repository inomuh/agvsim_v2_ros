cmake_minimum_required(VERSION 2.8.3)
project(agv2_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(agv2_laser_angular_bound_filter src/agv2_laser_angular_bound_filter.cpp)
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(agv2_laser_angular_bound_filter ${sensor_msgs_EXPORTED_TARGETS})
endif()
target_link_libraries(agv2_laser_angular_bound_filter ${catkin_LIBRARIES})


add_executable(agv2_initial_pose src/agv2_initial_pose.cpp)
target_link_libraries(agv2_initial_pose ${catkin_LIBRARIES})
