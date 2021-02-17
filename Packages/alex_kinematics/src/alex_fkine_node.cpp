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
#include <tf2/transform_datatypes.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include "alex_kinematics/alex_fkine.h"
#include "alex_global/global_definitions.h"
//#include "alex_kinematics/alex_fkine_node.h"

double distance(double x1, double y1, double x2, double y2) {
  double dist = abs(sqrt(pow((y1 - y2), 2) + pow(x1 - x2, 2)));
  return dist;
}

double angleCosineRule(double a, double b, double c) {
  double A = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
  return A;
}

double sideCosineRule(double b, double c, double A) {
  double a = sqrt(pow(b, 2) + pow(c, 2) - 2 * b * c * cos(A));
  return a;
}

tf2::Quaternion quatConversion(geometry_msgs::Quaternion q) {
  tf2::Quaternion Q(q.x, q.y, q.z, q.w);
  return Q;
}

geometry_msgs::Quaternion quatConversion(tf2::Quaternion q) {
  geometry_msgs::Quaternion Q;
  Q.x = q.x();
  Q.y = q.y();
  Q.z = q.z();
  Q.w = q.w();
  return Q;
}

geometry_msgs::Quaternion setRPY(tf2Scalar& roll, tf2Scalar& pitch, tf2Scalar& yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return quatConversion(q);
}

void getRPY(tf2::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

void getRPY(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf2::Quaternion Q = quatConversion(q);
  getRPY(Q, roll, pitch, yaw);
}

bool legFkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  // 1.0 LEFT LEG
  // 1.1 Knee/Shin Mechanism
  double roll, pitch, yaw;
  getRPY(transforms[prefix + "_knee_link_b"].transform.rotation, roll, pitch, yaw);
  double q2 = yaw;
  double localX2 = l_knee_b * cos(q2);
  double localY2 = -l_knee_b * sin(q2);

  getRPY(transforms[prefix + "_knee_link_a"].transform.rotation, roll, pitch, yaw);
  double q1 = yaw + q2;
  double localX1 = l_knee_a * cos(q1);
  double localY1 = -l_knee_a * sin(q1);


  double lr1 = distance(localX1, localY1, localX2, localY2);
  double alpha1 = angleCosineRule(l_knee_a, l_knee_b, lr1);
  double alpha2 = angleCosineRule(l_shin_a, lr1, l_shin_connection);
  double gamma1 = angleCosineRule(lr1, l_knee_b, l_knee_a);
  double gamma2 = angleCosineRule(lr1, l_shin_connection, l_shin_a);
  double beta1 = angleCosineRule(l_knee_b, lr1, l_knee_a);
  double beta2 = angleCosineRule(l_shin_connection, lr1, l_shin_a);
  double alpha = alpha1 + alpha2;
  double beta = beta1 + beta2;
  double q3 = sqrt((M_PI - beta)/M_PI);
  double q4 = -sqrt((M_PI - alpha)/M_PI);

  yaw = (M_PI - beta);
  pitch = 0;
  roll = 0;

  transforms[prefix + "_shin_link_a"].transform.rotation = setRPY(roll, pitch, yaw);
  yaw = (alpha - M_PI);


  transforms[prefix + "_shin_link_b"].transform.rotation = setRPY(roll, pitch, yaw);
  tf2Scalar zero = 0;
  transforms[prefix + "_shin_link_connection"].transform.rotation = setRPY(zero, zero, zero);

  // 1.2 Shin/Ankle Mechanism
  geometry_msgs::TransformStamped left_foot_offset, ankle_b_offset, ankle_c_offset;

  left_foot_offset.transform.translation.x = 0;
  left_foot_offset.transform.translation.y = 0;
  left_foot_offset.transform.translation.x += l_ankle_connection + (l_shin_connection - l_ankle_connection) + (l_shin_b - l_shin_connection);

  ankle_b_offset.transform.translation.x = 0;
  ankle_b_offset.transform.translation.y = 0;
  ankle_b_offset.transform.translation.x += l_ankle_connection;
  getRPY(transforms[prefix + "_ankle_link_a"].transform.rotation, roll, pitch, yaw);
  ankle_b_offset.transform.translation.x += l_ankle_a * cos(yaw);
  ankle_b_offset.transform.translation.y -= l_ankle_a * sin(yaw);

  ankle_c_offset = ankle_b_offset;
  double yaw2;
  getRPY(transforms[prefix + "_ankle_link_b"].transform.rotation, roll, pitch, yaw2);
  ankle_c_offset.transform.translation.x += l_ankle_b * cos(yaw + yaw2);
  ankle_c_offset.transform.translation.y -= l_ankle_b * sin(yaw + yaw2);

  lr1 = distance(left_foot_offset.transform.translation.x, left_foot_offset.transform.translation.y, ankle_b_offset.transform.translation.x, ankle_b_offset.transform.translation.y);
  double lr2 = distance(left_foot_offset.transform.translation.x, left_foot_offset.transform.translation.y, ankle_c_offset.transform.translation.x, ankle_c_offset.transform.translation.y);
  alpha1 = angleCosineRule((l_shin_connection - l_ankle_connection) + (l_shin_b - l_shin_connection), l_ankle_a, lr1);
  alpha2 = angleCosineRule(lr2, l_ankle_b, lr1);
  beta1 = angleCosineRule(l_foot_a, (transforms[prefix + "_foot_link_a"].transform.translation.x + transforms[prefix + "_ankle_link_c_2"].transform.translation.x), lr2);
  beta2 = angleCosineRule(lr1, l_ankle_b, lr2);

  roll = 0;
  pitch = 0;

  double lr1Angle = (ankle_b_offset.transform.translation.y - left_foot_offset.transform.translation.y)/(ankle_b_offset.transform.translation.x - left_foot_offset.transform.translation.x);
  double lr2Angle = (ankle_c_offset.transform.translation.y - left_foot_offset.transform.translation.y)/(ankle_c_offset.transform.translation.x - left_foot_offset.transform.translation.x);
  if (lr2Angle > lr1Angle) {
    yaw = -(M_PI - (beta1 + beta2));

  } else {
    yaw = -(M_PI - (beta1 - beta2));

  }
  transforms[prefix + "_ankle_link_c_1"].transform.rotation = setRPY(roll, pitch, yaw);

  double omega = angleCosineRule(lr2, (transforms[prefix + "_foot_link_a"].transform.translation.x + transforms[prefix + "_ankle_link_c_2"].transform.translation.x), l_foot_a);
  yaw = -(M_PI - omega);
  transforms[prefix + "_foot_link_a"].transform.rotation = setRPY(roll, pitch, yaw);

  return false;
}

bool fkine(alex_kinematics::alex_fkine::Request &req, alex_kinematics::alex_fkine::Response &res) {
  std::vector<geometry_msgs::TransformStamped> tf_transforms_in = req.transforms;
  std::vector<geometry_msgs::TransformStamped> tf_transforms_out;
  std::map<std::string, geometry_msgs::TransformStamped> transformMap;

  for (auto x : tf_transforms_in) {
    transformMap[x.child_frame_id] = x;
  }

  legFkine("left", transformMap);
  legFkine("right", transformMap);

  for (auto x : transformMap) {
    tf_transforms_out.push_back(x.second);
  }
  res.transforms = tf_transforms_out;

  return true;
}

// bool fkine(alex_kinematics::fkine::Request &req, alex_kinematics::fkine::Response &res) {

// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_fkine_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("alex_fkine", fkine);
  ROS_INFO("Alex Fkine Node");
  ros::spin();

  return 0;
}
