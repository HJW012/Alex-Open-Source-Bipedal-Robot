#include <ros/ros.h>
#include <alex_global/global_definitions.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <alex_kinematics/alex_ikine.h>

ros::ServiceClient ikineClient;
alex_kinematics::alex_ikine ikineSrv;

int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "alex_ikine_test_node");
  ros::NodeHandle node;



  ikineClient = node.serviceClient<alex_kinematics::alex_ikine>("alex_ikine");

  ros::spin();

  return 0;
}
