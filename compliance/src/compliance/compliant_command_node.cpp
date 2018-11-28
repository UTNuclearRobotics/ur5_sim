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
