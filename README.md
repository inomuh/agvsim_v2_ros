# agvsim_v2_ros

First download agvsim_v2_ros metapackage

Exract ~/agvsim_v2_ros/agv2_description/meshes/22-10-2019-OTAv07/OTA-v0.7.stl.zip

sudo apt-get install ros-kinetic-navigation

sudo apt-get install ros-kinetic-gmapping

sudo apt-get install ros-kinetic-timed-roslaunch

sudo apt-get install ros-kinetic-dynamic-reconfigure

sudo apt-get install ros-kinetic-smach-viewer



gedit ~/.bashrc

export GAZEBO_MODEL_PATH=~/<WORKSPACE_NAME> /src/agvsim_v2_ros/building_editor_models: ~/<WORKSPACE_NAME> /src/agvsim_v2_ros/model_editor_models:$GAZEBO_MODEL_PATH



cd <WORKSPACE_NAME>

catkin_make

catkin_make install

$ roslaunch agvsim_v2_start start_agv.launch
