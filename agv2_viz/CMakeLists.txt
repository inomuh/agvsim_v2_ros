cmake_minimum_required(VERSION 2.8.3)
project(agv2_viz)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

foreach(dir launch rviz)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
