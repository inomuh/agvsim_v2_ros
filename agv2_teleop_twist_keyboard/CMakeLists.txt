cmake_minimum_required(VERSION 2.8.3)
project(agv2_teleop_twist_keyboard)

find_package(catkin REQUIRED)

catkin_package()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
	src/teleop_twist_keyboard.py
	src/teleop_twist_keyboard_hakan.py
	src/teleop_twist_keyboard_agv1.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

