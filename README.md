universal_robot
======

UR5 Simulation Usage:

./ur_gazebo/scripts/gazebo_starter.sh

roslaunch ur_gazebo jog_arm.launch


UR5 Initial simulation setup:
======

(This is a hack -- Gazebo must be paused initially to set the robot joints. See https://answers.ros.org/question/248178/can-i-set-initial-joint-positions-in-gazebomoveit-via-configuration/ ) 

Copy custom sensor models from ur_gazebo/models/_/_.sdf to ~/.gazebo/models/_/_.sdf

To use with the SpaceNavigator, disable Gazebo's link to the SpaceNavigator:

http://answers.gazebosim.org/question/14225/how-can-i-turn-off-the-space-navigators-control-of-the-camera/


UR3 Hardware usage:
======

roslaunch ur_modern_driver ur3_ros_control.launch robot_ip:=192.168.1.102
(OR)
roslaunch ur_modern_driver ur3_bringup_joint_limited.launch robot_ip:=192.168.1.102

roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch

roslaunch ur3_moveit_config moveit_rviz.launch config:=true


Other useful stuff:
======

rosservice call /controller_manager/switch_controller "start_controllers:
- 'compliance_controller'
stop_controllers:
- 'vel_based_pos_traj_controller'
strictness: 1"