#!/bin/bash

echo "###############   LOADING ROBOT DESCRIPTION   ###########"
gnome-terminal -e 'roslaunch ur_description temoto_ur5_upload.launch limited:="true"'

sleep 5
echo "###############   STARTING GAZEBO PAUSED   ###############" 
gnome-terminal -e "roslaunch ur_gazebo start_gazebo_paused.launch"

sleep 5
echo "###############   LOAD CONTROLLERS   ###############"
gnome-terminal -e "roslaunch ur_gazebo load_controllers.launch"

sleep 2
echo "###############   SPAWNING MODEL   ###############"
gnome-terminal -e "roslaunch ur_gazebo spawn_model.launch"

sleep 2
echo "###############   SETTING JOINT STATES   ###############"
rosservice call /gazebo/set_model_configuration "model_name: 'robot' 
urdf_param_name: 'robot_description'
joint_names:
- 'elbow_joint'
joint_positions:
- 0.9"

sleep 2
echo "###############   UNPAUSING PHYSICS   ###############"
rosservice call /gazebo/unpause_physics "{}"

sleep 2
echo "###############   START CONTROLLERS   ###############"
gnome-terminal -e "roslaunch ur_gazebo start_controllers.launch"
