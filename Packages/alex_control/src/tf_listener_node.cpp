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
#include <alex_global/global_definitions.h>


// tf_broadcaster_.sendTransform(tf_transforms);
bool temp = false;
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
  double null1, null2, null3;
  alex_msgs::MotorParamOut m1;
  getRPY(msg->transforms[1].transform.rotation, null1, null2, null3);
  m1.id = 5;
  m1.pose = -(double(null3 - 3*M_PI/4));
  double tempVal1 = -(double(null3 - 3*M_PI/4));
  m1.kd = 2;
  m1.kp = 50;
  //std::cout << "M0 Pose: " << msg->transforms[1].child_frame_id << std::endl;
  alex_msgs::MotorParamOut m0;
  getRPY(msg->transforms[0].transform.rotation, null1, null2, null3);
  m0.id = 4;
  m0.speed = 0;
  m0.pose = -(double(null3) - M_PI/4) - double(m1.pose);
  double tempVal0 =  -(double(null3) - M_PI/4) - double(m1.pose);
  m0.kd = 2;
  m0.kp = 50;

  tfScalar null11 = 0;
  tfScalar null22 = 0;

  // std::cout << "M1 Pose: " << tempVal1 << std::endl;
  tfScalar null33 = tfScalar(m0.pose);
  geometry_msgs::TransformStamped m0TF, m1TF;

  tf2_ros::TransformBroadcaster test;
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  m0TF.transform.rotation = setRPY(null11, null22, null33);
  m0TF.header.stamp = ros::Time::now();
  m0TF.header.frame_id = "base_link";
  m0TF.child_frame_id = "M0";
  null33 = tfScalar(m1.pose);
  m1TF.transform.rotation = setRPY(null11, null22, null33);
  m1TF.header.stamp = m0TF.header.stamp;
  m1TF.header.frame_id = "base_link";
  m1TF.child_frame_id = "M1";
  tf_transforms.push_back(m0TF);
  tf_transforms.push_back(m1TF);
  if (!temp) {
    temp = true;
    test.sendTransform(tf_transforms);
  }
  if (msg->transforms[0].child_frame_id == "link1" && msg->transforms[1].child_frame_id == "link2" ) {
    std::cout << "NOW" << std::endl;
    test.sendTransform(tf_transforms);
  }
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
