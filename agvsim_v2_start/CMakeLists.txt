cmake_minimum_required(VERSION 2.8.3)
project(agvsim_v2_start)

find_package(catkin REQUIRED)


catkin_package()

foreach(dir launch scripts)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()