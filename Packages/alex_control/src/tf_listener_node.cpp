#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "alex_driver/send_tmotor_command.h"
#include "alex_msgs/MotorParamOut.h"

#include <map>
#include <string>

#include <ros/ros.h>
#include <urdf/model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

ros::ServiceClient client;

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

void TFCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
  // transforms 0 and 1 are the ones we care about (link 1 and link 2)
  alex_msgs::MotorParamOut m0;
  double null1, null2, null3;
  getRPY(msg->transforms[0].transform.rotation, null1, null2, null3);
  m0.id = 0;
  m0.pose = double(null3) - M_PI/4;
  m0.kd = 200;
  m0.kp = 4;

  alex_msgs::MotorParamOut m1;
  getRPY(msg->transforms[1].transform.rotation, null1, null2, null3);
  m1.id = 1;
  m1.pose = double(null3) - 3*M_PI/4;
  std::cout << m1.pose << std::endl;
  m1.kd = 200;
  m1.kp = 4;

  alex_driver::send_tmotor_command srv;
  srv.request.motorParamOut = m0;
  client.call(srv);

  srv.request.motorParamOut = m1;
  client.call(srv);
}

int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "alex_tf_listener_node");
  ros::NodeHandle n;



  ros::Subscriber TFListener = n.subscribe("tf", 1000, TFCallback);
  client = n.serviceClient<alex_driver::send_tmotor_command>("send_tmotor_command");
  ros::spin();

  return 0;
}
