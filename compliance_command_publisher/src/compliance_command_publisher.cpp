#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

// Send a small velocity to the joints, for testing.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compliance_command_publisher");

  ros::NodeHandle n;

  ros::Publisher compliance_command_pub = n.advertise<std_msgs::Float64MultiArray>(
    "/compliance_controller/compliance_velocity_adjustment", 1);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    std_msgs::Float64MultiArray joint_velocities;
    joint_velocities.data.resize(6);
    for (std::size_t i=0; i<joint_velocities.data.size(); ++i)
      joint_velocities.data[i] = 0.001;


    compliance_command_pub.publish(joint_velocities);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
