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

std::map<std::string, geometry_msgs::TransformStamped> transformMap;

// tf_broadcaster_.sendTransform(tf_transforms);
bool temp = false;
ros::ServiceClient client;

void mapTransforms(const tf2_msgs::TFMessage::ConstPtr& msg, std::map<std::string, geometry_msgs::TransformStamped>& transformMap) {
  for (auto x : msg->transforms) {
    transformMap[x.child_frame_id] = x;
  }
}

void TFCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
  mapTransforms(msg, transformMap);
  // transforms 0 and 1 are the ones we care about (link 1 and link 2)
  double null1, null2, null3;
  alex_msgs::MotorParamOut m1;
  getRPY(transformMap["left_knee_link_b"].transform.rotation, null1, null2, null3);
  m1.id = 5;
  m1.pose = (M_PI/2 + null2) - M_PI/4;
  double tempVal1 = -(double(null2 - 3*M_PI/4));
  m1.kd = 2;
  m1.kp = 50;
  std::cout << "M1 Raw: " << null2 << std::endl;
  std::cout << "M1 Pose: " << m1.pose << std::endl;
  //std::cout << "M0 Pose: " << msg->transforms[1].child_frame_id << std::endl;
  alex_msgs::MotorParamOut m0;
  getRPY(transformMap["left_knee_link_a"].transform.rotation, null1, null2, null3);
  m0.id = 4;
  m0.speed = 0;
  m0.pose = abs(null3) - M_PI/2;
  double tempVal0 =  -(double(null3) - M_PI/4) - double(m1.pose);
  std::cout << "M0 Raw: " << null3 << std::endl;
  std::cout << "M0 Pose: " << m0.pose << std::endl;
  m0.kd = 2;
  m0.kp = 50;

  tfScalar null11 = 0;
  tfScalar null22 = 0;


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
