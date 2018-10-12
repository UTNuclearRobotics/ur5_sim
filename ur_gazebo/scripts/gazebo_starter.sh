#!/bin/bash

echo "###############   LOADING ROBOT DESCRIPTION   ###########"
gnome-terminal -e 'roslaunch ur_description temoto_ur5_upload.launch limited:="true"'

sleep 3
echo "###############   STARTING GAZEBO PAUSED   ###############" 
gnome-terminal -e "roslaunch ur_gazebo start_gazebo_paused.launch"


sleep 5
echo "###############   SPAWNING MODEL   ###############"
gnome-terminal -e "roslaunch ur_gazebo spawn_model.launch"

sleep 3
echo "###############   RESETTING SIMULATION   ###############"
rosservice call /gazebo/reset_simulation "{}"

sleep 1
echo "###############   SETTING JOINT STATES   ###############"
rosservice call /gazebo/set_model_configuration "model_name: 'robot' 
urdf_param_name: 'robot_description'
joint_names:
- 'shoulder_pan_joint'
- 'shoulder_lift_joint'
- 'elbow_joint'
- 'wrist_1_joint'
- 'wrist_2_joint'
- 'wrist_3_joint'

joint_positions:
- 0.0
- -1.0
- 1.0
- 1.0
- 1.0
- -1.5
"
# Adjust the bookshelf
rosservice call /gazebo/set_model_state "
model_state:
  model_name: 'bookshelf_0'
  pose:
    position:
      x: -0.42
      y: 0.85
    orientation:
      z: 0.343
      w: 0.939
"

# Adjust the drill
rosservice call /gazebo/set_model_state "
model_state:
  model_name: 'drill_0'
  pose:
    position:
      x: -0.169
      y: 0.65
      z: 0.4379
    orientation:
      z: 0.343
      w: 0.939
"

# Adjust the camera
rosservice call /gazebo/set_link_state "
link_state:
  link_name: 'asus_world_0::link'
  pose:
    position:
      x: -1
      y: -2
      z: 1
    orientation:
      z: 0.5
      w: 0.866
"

#sleep 1
#echo "###############   LOAD CONTROLLERS   ###############"
#gnome-terminal -e "roslaunch ur_gazebo load_controllers.launch"

#sleep 1
#echo "###############   START CONTROLLERS   ###############"
#gnome-terminal -e "roslaunch ur_gazebo start_controllers.launch"

#sleep 2
#echo "###############   UNPAUSING PHYSICS   ###############"
#rosservice call /gazebo/unpause_physics "{}"

#sleep 2
#echo "###############   Moveit rviz etc   ###############"
#gnome-terminal -e "roslaunch ur_gazebo master.launch"
