#include "ros/ros.h"
#include <map>
#include <string>

#include <urdf/model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include "alex_kinematics/alex_ikine.h"
//#include "alex_kinematics/alex_fkine_node.h"

bool ikine(alex_kinematics::alex_ikine::Request &req, alex_kinematics::alex_ikine::Response &res) {

}

// bool fkine(alex_kinematics::fkine::Request &req, alex_kinematics::fkine::Response &res) {

// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_ikine_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("alex_ikine_node", ikine);
  ROS_INFO("Alex Ikine Node");
  ros::spin();

  return 0;
}
