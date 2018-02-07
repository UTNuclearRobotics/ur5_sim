universal_robot
======

Usage:

1) roslaunch ur_gazebo temoto_master.launch

2) roslaunch ur_gazebo temoto_ur5_run_last.launch

(This is a hack -- Gazebo must be paused initially to set the robot joints. See https://answers.ros.org/question/248178/can-i-set-initial-joint-positions-in-gazebomoveit-via-configuration/ ) 

Reference:

http://wiki.ros.org/universal_robot

Copy custom sensor models from ur_gazebo/models/_/_.sdf to ~/.gazebo/models/_/_.sdf

To use with the SpaceNavigator, disable Gazebo's link to the SpaceNavigator:

http://answers.gazebosim.org/question/14225/how-can-i-turn-off-the-space-navigators-control-of-the-camera/