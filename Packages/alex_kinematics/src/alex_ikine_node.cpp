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
#include "alex_global/global_definitions.h"

//#include "alex_kinematics/alex_fkine_node.h"
double distance(double, double, double, double);
double angleCosineRule(double, double, double);
double sideCosineRule(double, double, double);
geometry_msgs::Quaternion setRPY(tf2Scalar&, tf2Scalar&, tf2Scalar&);
void getRPY(tf2::Quaternion, double&, double&, double&);
void getRPY(geometry_msgs::Quaternion, double&, double&, double&);
tf2::Quaternion quatConversion(geometry_msgs::Quaternion);
geometry_msgs::Quaternion quatConversion(tf2::Quaternion);
bool ikine(alex_kinematics::alex_ikine::Request &req, alex_kinematics::alex_ikine::Response &res);
bool legIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms);
bool hipIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms);
bool kneeIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms);
bool ankleIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms);
geometry_msgs::TransformStamped getRelativeTransform(geometry_msgs::TransformStamped targetTF, geometry_msgs::TransformStamped parentTF);


bool ikine(alex_kinematics::alex_ikine::Request &req, alex_kinematics::alex_ikine::Response &res) {
  std::map<std::string, geometry_msgs::TransformStamped> transforms;
  for (auto x : req.footTransforms) {
    transforms[x.child_frame_id] = x;
  }
  legIkine("left", transforms);
  legIkine("right", transforms);
}

bool legIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  std::map<std::string, geometry_msgs::TransformStamped> mappedTransforms = transforms;
  hipIkine(prefix, transforms, mappedTransforms);   // Find hip joint angle and map foot coords to calculate knee/ankle joint angles
  kneeIkine(prefix, transforms, mappedTransforms);
  ankleIkine(prefix, transforms, mappedTransforms);

}

//Ikine for hip joint angles and to map foot coords - must be performed before knee ikine
bool hipIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms) {

}

//Ikine for knee joint angles - must be performed after hip ikine with mapped foot coords
bool kneeIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms) {

}

// Ikine for ankle - must be performed after hip and knee ikine with mapped foot coords
bool ankleIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms) {

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
