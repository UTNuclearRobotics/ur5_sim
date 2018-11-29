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

      // Transform this Cartesian velocity to the MoveIt! planning frame
      geometry_msgs::TransformStamped force_torque_to_moveit_tf;
      while (force_torque_to_moveit_tf.header.frame_id == "" && ros::ok())
      {
        try{
          force_torque_to_moveit_tf = tf_buffer_.lookupTransform(force_torque_frame_name_, moveit_planning_frame_name_,
                                   ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ROS_WARN("Waiting for the transform from force/torque to moveit_planning_frame to be published.");
          ros::Duration(0.01).sleep();
          continue;
        }
      }

      // Multiply by the Jacobian pseudo-inverse to calculate a joint velocity vector

      // Publish this joint velocity vector

      ROS_INFO_STREAM(velocity.at(0));
    }
  }
}