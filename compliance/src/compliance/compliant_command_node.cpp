// Subscribe to a wrench and publish a compliant joint velocity correction

#include <compliance/compliant_command_node.h>

// Initialize static member of class PublishComplianceJointVelocities
compliant_command_node::DefaultComplianceParameters compliant_command_node::PublishComplianceJointVelocities::compliance_params_;

int main(int argc, char** argv) {
  ros::init(argc, argv, "compliant_command_node");

  // Do compliance calculations in this class
  compliant_command_node::PublishComplianceJointVelocities publish_compliance_velocities;

  // Spin and publish compliance velocities, unless disabled by a service call
  publish_compliance_velocities.spin();

  return 0;
}

void compliant_command_node::PublishComplianceJointVelocities::spin()
{
  while (ros::ok())
  {
    ros::spinOnce();
    // TODO: do not hard-code this spin rate
    ros::Duration(0.01).sleep();

    if (compliance_enabled_)
    {
      // The algorithm:
      // Get a wrench in the force/torque sensor frame
      // With the compliance object, calculate a compliant, Cartesian velocity in the force/torque sensor frame
      // Transform this Cartesian velocity to the MoveIt! planning frame
      // Multiply by the Jacobian pseudo-inverse to calculate a joint velocity vector
      // Publish this joint velocity vector
      // Another node can sum it with nominal joint velocities to result in spring-like motion

      // Input to the compliance calculation is an all-zero nominal velocity
      std::vector<double> velocity(6);
      // Calculate the compliant velocity adjustment
      compliant_control_instance_.getVelocity(velocity, last_wrench_data_, velocity);

      geometry_msgs::Vector3Stamped translational_velocity;
      translational_velocity.header.frame_id = force_torque_frame_name_;
      translational_velocity.vector.x = velocity[0];
      translational_velocity.vector.y = velocity[1];
      translational_velocity.vector.z = velocity[2];

      geometry_msgs::Vector3Stamped rotational_velocity;
      rotational_velocity.header.frame_id = force_torque_frame_name_;
      rotational_velocity.vector.x = velocity[3];
      rotational_velocity.vector.y = velocity[4];
      rotational_velocity.vector.z = velocity[5];      

      // Transform this Cartesian velocity to the MoveIt! planning frame
      // TODO: get the MoveIt! planning frame programmatically
      geometry_msgs::TransformStamped force_torque_to_moveit_tf;
      while (force_torque_to_moveit_tf.header.frame_id == "" && ros::ok())
      {
        try
        {
          force_torque_to_moveit_tf = tf_buffer_.lookupTransform(
            moveit_planning_frame_name_,
            force_torque_frame_name_,
            ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s",ex.what());
          ROS_WARN("Waiting for the transform from force/torque to moveit_planning_frame to be published.");
          ros::Duration(0.01).sleep();
          continue;
        }
      }
      tf2::doTransform(translational_velocity, translational_velocity, force_torque_to_moveit_tf);
      tf2::doTransform(rotational_velocity, rotational_velocity, force_torque_to_moveit_tf);

      Eigen::VectorXd cartesian_velocity(6);
      cartesian_velocity[0] = translational_velocity.vector.x;
      cartesian_velocity[1] = translational_velocity.vector.y;
      cartesian_velocity[2] = translational_velocity.vector.z;
      cartesian_velocity[3] = rotational_velocity.vector.x;
      cartesian_velocity[4] = rotational_velocity.vector.y;
      cartesian_velocity[5] = rotational_velocity.vector.z;

      // Multiply by the Jacobian pseudo-inverse to calculate a joint velocity vector
      Eigen::MatrixXd j = kinematic_state_->getJacobian(joint_model_group_);
      // J^T* (J*(J^T)^-1) is the Moore-Penrose pseudo-inverse
      Eigen::VectorXd delta_theta = j.transpose() * (j * j.transpose()).inverse() * cartesian_velocity;
      ROS_WARN_STREAM(delta_theta);

      // Publish this joint velocity vector
      // Type is std_msgs/Float64MultiArray.h
      std_msgs::Float64MultiArray delta_theta_msg;
      for (int i=0; i<delta_theta.size(); ++i)
      {
        delta_theta_msg.data.push_back( delta_theta[i] );
      }
      compliant_velocity_pub_.publish(delta_theta_msg);
    }
  }
}