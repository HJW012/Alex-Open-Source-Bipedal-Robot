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

  ros::Publisher jointStatePub = node.advertise<sensor_msgs::JointState>("alex_ikine_joint_states", 1000);

  ikineClient = node.serviceClient<alex_kinematics::alex_ikine>("alex_ikine");
  geometry_msgs::TransformStamped target_left_foot_a;
  target_left_foot_a.child_frame_id = "left_foot_a";
  target_left_foot_a.transform.translation.y = -0.2;
  target_left_foot_a.transform.translation.z = -0.4;

while (ros::ok()) {
  target_left_foot_a.child_frame_id = "left_foot_a";
  sensor_msgs::JointState testJS;
  std::vector<std::string>name;
  std::vector<double>position;
  name.push_back("base_link_to_right_hip_link");
  position.push_back(1.2);

  ikineSrv.request.footTransforms.push_back(target_left_foot_a);
  ikineClient.call(ikineSrv);
  testJS.name = name;

  testJS.name = ikineSrv.response.jointStates.name;
  testJS.position = ikineSrv.response.jointStates.position;
  jointStatePub.publish(testJS);


  ros::spinOnce();
}
  return 0;
}
