#include "ros/ros.h"
#include <map>
#include <string>

#define l1 0.1f
#define l2 0.3f
#define l3 0.309f
#define l4 0.115f
#define l5 0.19f

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
#include "alex_kinematics/alex_leg_fkine.h"
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

tf::Quaternion quatConversion(geometry_msgs::Quaternion q) {
  tf::Quaternion Q(q.x, q.y, q.z, q.w);
  return Q;
}

geometry_msgs::Quaternion quatConversion(tf::Quaternion q) {
  geometry_msgs::Quaternion Q;
  Q.x = q.x();
  Q.y = q.y();
  Q.z = q.z();
  Q.w = q.w();
  return Q;
}

geometry_msgs::Quaternion setRPY(tfScalar& roll, tfScalar& pitch, tfScalar& yaw) {
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return quatConversion(q);
}

void getRPY(tf::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

void getRPY(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf::Quaternion Q = quatConversion(q);
  getRPY(Q, roll, pitch, yaw);
}


bool fkine(alex_kinematics::alex_leg_fkine::Request &req, alex_kinematics::alex_leg_fkine::Response &res) {
  geometry_msgs::TransformStamped tf1;
  geometry_msgs::TransformStamped tf2;
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  for (auto tf : req.jointAngles) {
    if (tf.header.frame_id == "base_link" && tf.child_frame_id == "link1") {
      tf1 = tf;
    } else if (tf.header.frame_id == "base_link" && tf.child_frame_id == "link2") {
      tf2 = tf;
    }
  }
  tf1.header.stamp = ros::Time::now();
  tf2.header.stamp = ros::Time::now();
  tf_transforms.push_back(tf1);
  tf_transforms.push_back(tf2);


  double roll, pitch, yaw;
  getRPY(tf1.transform.rotation, roll, pitch, yaw);
  double q1 = yaw;
  double localX1 = l1 * cos(q1);
  double localY1 = -l1 * sin(q1);

  getRPY(tf2.transform.rotation, roll, pitch, yaw);
  double q2 = yaw;
  double localX2 = l2 * cos(q2);
  double localY2 = -l2 * sin(q2);

  double lr1 = distance(localX1, localY1, localX2, localY2);
  double alpha1 = angleCosineRule(l1, l2, lr1);
  double alpha2 = angleCosineRule(l3, lr1, l4);
  double gamma1 = angleCosineRule(lr1, l2, l1);
  double gamma2 = angleCosineRule(lr1, l4, l3);
  double beta1 = angleCosineRule(l2, lr1, l1);
  double beta2 = angleCosineRule(l4, lr1, l3);
  double alpha = alpha1 + alpha2;
  double beta = beta1 + beta2;
  double q3 = sqrt((M_PI - beta)/M_PI);
  double q4 = -sqrt((M_PI - alpha)/M_PI);

  yaw = (M_PI - beta);
  pitch = 0;
  roll = 0;
  geometry_msgs::TransformStamped tempTF;
  tempTF.header.frame_id = "link1";
  tempTF.child_frame_id = "link3";
  tempTF.transform.rotation = setRPY(roll, pitch, yaw);
  tempTF.transform.translation.x = l1;
  tempTF.header.stamp = tf1.header.stamp;
  tf_transforms.push_back(tempTF);
  yaw = (alpha - M_PI);


  tempTF.transform.rotation = setRPY(roll, pitch, yaw);
  tempTF.header.frame_id = "link2";
  tempTF.child_frame_id = "link4";
  tempTF.transform.translation.x = l2;

  tf_transforms.push_back(tempTF);
  tfScalar zero = 0;
  tempTF.transform.rotation = setRPY(zero, zero, zero);
  tempTF.header.frame_id = "link4";
  tempTF.child_frame_id = "link5";
  tempTF.transform.translation.x = l4;
  tf_transforms.push_back(tempTF);

  tempTF.transform.rotation = setRPY(zero, zero, zero);
  tempTF.header.frame_id = "link5";
  tempTF.child_frame_id = "foot";
  tempTF.transform.translation.x = l5;
  tf_transforms.push_back(tempTF);


  res.transforms = tf_transforms;
}

// bool fkine(alex_kinematics::fkine::Request &req, alex_kinematics::fkine::Response &res) {

// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_leg_fkine_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("alex_leg_fkine_node", fkine);
  ROS_INFO("Alex Leg Fkine Node");
  ros::spin();

  return 0;
}
