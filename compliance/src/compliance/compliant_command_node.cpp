// Subscribe to a wrench and publish a compliant joint velocity correction

#include <compliance/compliant_control.h>
#include <ros/ros.h>

namespace compliant_command_node
{

struct ComplianceParameters
{
  std::vector<double> stiffness{4000, 4000, 4000, 4000, 4000, 4000};
  // Related to the cutoff frequency of the low-pass filter.
  double filter_param = 10.;
  // Deadband for force/torque measurements
  std::vector<double> deadband{10, 10, 10, 10, 10, 10};
  // Stop when force exceeds X N or torque exceeds X Nm in any dimension.
  // The robot controller's built-in safety limits are ~90 N, ? Nm
  std::vector<double> end_condition_wrench{80, 80, 80, 60, 60, 60};
  // Highest allowable force/torque across all dimensions.
  double highest_allowable_force = 88, highest_allowable_torque = 50;
  // Current force/torque data
  geometry_msgs::WrenchStamped force_torque_data;
  // Outgoing velocity msg
  std::vector<double> velocity_out{0, 0, 0, 0, 0, 0};
};

class PublishComplianceJointVelocities
{
public:
  PublishComplianceJointVelocities(geometry_msgs::WrenchStamped &bias) :
    compliant_control_instance_(
      compliance_params_.stiffness,
      compliance_params_.deadband,
      compliance_params_.end_condition_wrench,
      compliance_params_.filter_param,
      bias,
      compliance_params_.highest_allowable_force,
      compliance_params_.highest_allowable_torque)
  { 
  }


private:
  // An object to do compliance calculations
  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  compliant_control::CompliantControl compliant_control_instance_;

  ComplianceParameters compliance_params_;
};

} // namespace compliant_command_node

int main(int argc, char** argv) {
  ros::init(argc, argv, "compliant_command_node");

  ROS_INFO_STREAM("Hello, world!");

  return 0;
}
