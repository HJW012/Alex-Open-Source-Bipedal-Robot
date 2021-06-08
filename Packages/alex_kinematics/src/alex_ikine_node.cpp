#include <alex_kinematics/alex_ikine_node.h>

bool hipIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {

}

bool lowerLegIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {

}

bool ikine(alex_kinematics::alex_ikine::Request &req, alex_kinematics::alex_ikine::Response &res) {
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_ikine_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("alex_ikine", ikine);
  ROS_INFO("Alex Ikine Node");

  ros::spin();

  return 0;
}
