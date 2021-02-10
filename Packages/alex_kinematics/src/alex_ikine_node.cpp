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
#include "alex_kinematics/alex_ikine.h"
// Need to change these and URDF is limb lengths change
#define l1 0.1f
#define l2 0.3f
#define l3 0.309f
#define l4 0.115f
#define l5 0.19f
#define l6 0.02735f
#define l7 0.0432f
#define l8a 0.1493f
#define l8b 0.155795f
#define l9 0.055f
#define l10 0.04025f
#define l11 0.07475f

//#include "alex_kinematics/alex_fkine_node.h"
double distance(double, double, double, double);
double angleCosineRule(double, double, double);
double sideCosineRule(double, double, double);
geometry_msgs::Quaternion setRPY(tf2Scalar&, tf2Scalar&, tf2Scalar&);
void getRPY(tf2::Quaternion, double&, double&, double&);
void getRPY(geometry_msgs::Quaternion, double&, double&, double&);
tf2::Quaternion quatConversion(geometry_msgs::Quaternion);
geometry_msgs::Quaternion quatConversion(tf2::Quaternion);
bool legIkineNoHip(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>&);


bool ikine(alex_kinematics::alex_ikine::Request &req, alex_kinematics::alex_ikine::Response &res) {
  std::map<std::string, geometry_msgs::TransformStamped> transforms;
  for (auto x : req.transforms) {
    transforms[x.child_frame_id] = x;
  }
  legIkineNoHip("left", transforms);
  legIkineNoHip("right", transforms);
}

bool legIkineNoHip(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  // Perhaps the entered transform will be in world coordinates relative to hip or knee joint or even base link - only 2 or 4 transforms should be input
  double lr1 = distance(transforms[prefix + "_foot_b"].transform.translation.x, transforms[prefix + "_foot_b"].transform.translation.y, transforms[prefix + " _knee_joint"].transform.translation.x, transforms[prefix +"_knee_joint"].transform.translation.y);
  double alpha = angleCosineRule(lr1, l2, l4 + l5);
  double lr2 = angleCosineRule(l2, l4, alpha);
  double beta = angleCosineRule(lr2, l1, l3);
  double delta1 = angleCosineRule(l2, l4, lr2);
  double delta2 = angleCosineRule(l1, lr2, l3);
  double gamma1 = angleCosineRule(l4, l2, lr2);
  double gamma2 = angleCosineRule(l3, lr2, l1);
  double gamma = gamma1 + gamma2;
  double iota = angleCosineRule(l4 + l5, l2, lr1);
  double sigma = 0;
  if (transforms[prefix + "_foot_b"].transform.translation.x > transforms["base_link"].transform.translation.x) {
    sigma = -atan((transforms[prefix + "_foot_b"].transform.translation.y > transforms[prefix + " _knee_joint"].transform.translation.y)/(transforms[prefix + "_foot_b"].transform.translation.x > transforms[prefix + " _knee_joint"].transform.translation.x));
  } else {
    sigma = M_PI - atan((transforms[prefix + "_foot_b"].transform.translation.y > transforms[prefix + " _knee_joint"].transform.translation.y)/(transforms[prefix + "_foot_b"].transform.translation.x > transforms[prefix + " _knee_joint"].transform.translation.x));
  }

  std::vector<geometry_msgs::TransformStamped> jointAngles;
  tf2Scalar roll, pitch, yaw;
  pitch = (sigma + iota);
  roll = 0;
  yaw = 0;
  geometry_msgs::TransformStamped transform;
  transform.transform.rotation = setRPY(roll, pitch, yaw);
  jointAngles.push_back(transform);


}
/*

lr1 = distance(p0(1), p0(2), p4(1), p4(2));
    alpha = angleCosineRule(lr1, l(2), l(4) + l(5));
    lr2 = sideCosineRule(l(2), l(4), alpha);
    beta = angleCosineRule(lr2, l(1), l(3));
    delta1 = angleCosineRule(l(2), l(4), lr2);
    delta2 = angleCosineRule(l(1), lr2, l(3));
    gamma1 = angleCosineRule(l(4), l(2), lr2); % or PI - (alpha + delta1);
    gamma2 = angleCosineRule(l(3), lr2, l(1)); % or PI - (Beta + delta2);
    gamma = gamma1 + gamma2;
    iota = angleCosineRule(l(4) + l(5), l(2), lr1);

    if (p4(1) > p0(1))
        sigma = -atan((p4(2)-p0(2))/(p4(1)-p0(1)));
    else
        sigma = pi - atan((p4(2)-p0(2))/(p4(1)-p0(1)));
    end

    q(2) = sigma + iota;
    q(1) = q(2) - gamma;

    outP(1, 1) = p0(1) + l(1) * cos(q(1));
    outP(1, 2) = p0(2) - l(1) * sin(q(1));
    outP(2, 1) = p0(1) + l(2) * cos(q(2));
    outP(2, 2) = p0(2) - l(2) * sin(q(2));
    outP(3, 1) = p0(1) + lr2 * cos(q(1) + gamma2);
    outP(3, 2) = p0(2) - lr2 * sin(q(1) + gamma2);
    outP(4, 1) = p4(1);
    outP(4, 2) = p4(2);

    theta = q(2) - (pi - alpha);
    outP(5, 1) = outP(2, 1) + l(11) * cos(theta);
    outP(5, 2) = outP(2, 2) - l(11) * sin(theta);

*/

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_ikine_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("alex_ikine_node", ikine);
  ROS_INFO("Alex Ikine Node");
  ros::spin();

  return 0;
}
