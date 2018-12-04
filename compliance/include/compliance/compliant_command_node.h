
#ifndef COMPLIANT_COMMAND_NODE_H
#define COMPLIANT_COMMAND_NODE_H

#include <compliance/compliant_control.h>
#include <Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace compliant_command_node
{

struct ROSParameters
{
  double spin_rate, max_allowable_cmd_magnitude, low_pass_filter_param, highest_allowable_force, highest_allowable_torque;
  std::string jacobian_frame_name, force_torque_frame_name, force_torque_topic, move_group_name;
  std::vector<double> stiffness, deadband, end_condition_wrench;
};

class PublishCompliantJointVelocities
{
public:
  PublishCompliantJointVelocities() :
    tf_listener_(tf_buffer_)
  {
    readROSParameters();

    // Initialize the compliance object
    std::vector<double> stiffness = {
    };

  // Initialize an object of the compliance library
  // Assume a bias wrench of all zeros
  geometry_msgs::WrenchStamped bias;
  compliant_control_ptr_ = std::make_shared<compliant_control::CompliantControl>(
    compliance_params_.stiffness,
    compliance_params_.deadband,
    compliance_params_.end_condition_wrench,
    compliance_params_.low_pass_filter_param,
    bias,
    compliance_params_.highest_allowable_force,
    compliance_params_.highest_allowable_torque    
    );

  	enable_compliance_service_ = n_.advertiseService(
  	  n_.getNamespace() + "toggle_compliance_publication", &PublishCompliantJointVelocities::toggleCompliance, this);

    bias_compliance_service_ = n_.advertiseService(
      n_.getNamespace() + "bias_compliance_calcs", &PublishCompliantJointVelocities::biasCompliantCalcs, this);

    wrench_subscriber_ = n_.subscribe(compliance_params_.force_torque_topic, 1, &PublishCompliantJointVelocities::wrenchCallback, this);

    std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr_ = std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader);
    const robot_model::RobotModelPtr& kinematic_model = model_loader_ptr_->getModel();
    joint_model_group_ = kinematic_model->getJointModelGroup(compliance_params_.move_group_name);
    kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);

    compliant_velocity_pub_ = n_.advertise<std_msgs::Float64MultiArray>("/compliance_controller/compliance_velocity_adjustment", 1);

    joints_sub_ = n_.subscribe("joint_states", 1, &PublishCompliantJointVelocities::jointsCallback, this);
  }

  // Spin and publish compliance velocities, unless disabled by a service call
  void spin();

private:
  // A service callback. Toggles compliance publication on/off
  bool toggleCompliance(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res)
  {
    compliance_enabled_ = req.data;
    res.success = true;
    return true;
  }

  // A service callback. Biases (aka tares, aka zeroes) the compliance calculations
  // based on the most-recently-received wrench
  bool biasCompliantCalcs(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res)
  {
    if (req.data)
    {
      compliant_control_ptr_->biasSensor(last_wrench_data_);
      ROS_INFO_STREAM("The bias of compliance calculations was reset.");
      res.success = true;
    }
    else
      res.success = false;

    return true;
  }

  // Callback for wrench data
  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    last_wrench_data_ = *msg;
    last_wrench_data_.header.frame_id = force_torque_frame_name_;
  }

  // Callback for robot state updates
  void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    kinematic_state_->setVariableValues(*msg);
  }

  void readROSParameters();

  ros::NodeHandle n_;

  // An object to do compliance calculations.
  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  std::shared_ptr<compliant_control::CompliantControl> compliant_control_ptr_;


  bool compliance_enabled_ = true;

  static ROSParameters compliance_params_;

  // Publish compliance commands unless interrupted by a service call
  ros::ServiceServer enable_compliance_service_, bias_compliance_service_;

  // Subscribe to wrench data from a force/torque sensor
  ros::Subscriber wrench_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // TODO: do not hard-code these frame names
  std::string force_torque_frame_name_ = "base";
  std::string jacobian_frame_name_ = "base_link";

  // MoveIt! setup, required to retrieve the Jacobian
  const robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr kinematic_state_;

  geometry_msgs::WrenchStamped last_wrench_data_;

  ros::Publisher compliant_velocity_pub_;

  ros::Subscriber joints_sub_;
};

} // namespace compliant_command_node

#endif
