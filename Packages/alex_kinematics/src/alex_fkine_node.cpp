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


bool hipFkine2(std::string side, std::map<std::string, geometry_msgs::TransformStamped>& transformMap) {
  int reflect = 1;
  if (side == "left") {
    reflect = 1;
  } else if (side == "right") {
    reflect = -1;
  }

  std::map<std::string, geometry_msgs::TransformStamped> originalTransformMap = transformMap;
  double lr1;
  if (!Relative_Distance_In_Tree(transformMap, side + "_coronal_hip_pusher_link", side + "_coronal_hip_pivot_to_sagittal_hip_pivot_link", "base_link", lr1)) {
    std::cout << "Failing here1" << std::endl;

    return false;
  }

  //Need to get these from rosparam server
  double l_coronal_hip_push_point_to_coronal_hip_pivot = 0.067801;
  double l_coronal_hip_torque_arm_to_coronal_hip_push_point= 0.043656;
  double l_symbolic_hip_to_coronal_hip_pivot = 0.068139;
  double l_symbolic_hip_to_coronal_hip_rotor = 0.0338;
  double l_coronal_hip_torque_arm = 0.046803;
  double a_starting_angle1 = 84.802 * M_PI/180; //Angle in bottom right corner of 5 par hip linkage
  double a_starting_angle2 = 104.116 * M_PI/180; //ANgle between symbolic_to_hip_pivot and starting hip coronal pivot (horizontal)
  double starting_pivot_yaw = -M_PI/2;
  double templ2;
  if (!Relative_Distance_In_Tree(transformMap, side + "_coronal_hip_torque_arm_link", side + "_coronal_hip_pivot_to_sagittal_hip_pivot_link", "base_link", templ2)) {
    std::cout << "Failing here" << std::endl;
    return false;
  }

  double alpha1 = angleCosineRule(l_coronal_hip_push_point_to_coronal_hip_pivot, l_coronal_hip_torque_arm_to_coronal_hip_push_point, lr1);
  double beta1 = angleCosineRule(templ2, lr1, l_coronal_hip_torque_arm);

  double theta1 = -(M_PI - (alpha1 + beta1));
  double zero = 0;
  transformMap[side + "_coronal_hip_pusher_link"].transform.rotation = setRPY(zero, zero, theta1);
  double alpha2 = angleCosineRule(lr1, l_coronal_hip_torque_arm_to_coronal_hip_push_point, l_coronal_hip_push_point_to_coronal_hip_pivot);
  double theta2 = -(M_PI - alpha2);
  //theta2 = theta2 * reflect; //Handle left and right
  transformMap[side + "_coronal_hip_pusher_to_coronal_hip_pivot_link"].transform.rotation = setRPY(zero, zero, theta2);

  double alpha3 = angleCosineRule(l_coronal_hip_torque_arm_to_coronal_hip_push_point, lr1, l_coronal_hip_push_point_to_coronal_hip_pivot);
  double beta3 = angleCosineRule(l_coronal_hip_torque_arm, templ2, lr1);
  double gamma1 = angleCosineRule(l_symbolic_hip_to_coronal_hip_rotor, templ2, l_symbolic_hip_to_coronal_hip_pivot);
  //Alpha3 + beta3 + gamma1 should = a_starting_angle1 = 84.802
  double startingDifference = a_starting_angle2 - a_starting_angle1;
  double currentDifference = a_starting_angle2 - (alpha3 + beta3 + gamma1);
  double tempTheta = startingDifference - currentDifference;
  tempTheta = starting_pivot_yaw - tempTheta;
  transformMap[side + "_coronal_hip_pivot_to_sagittal_hip_pivot_link"].transform.rotation = setRPY(zero, zero, tempTheta);

  // std::cout << side << " lr1:" << lr1;
  // std::cout << side << " alpha1:" << alpha1 << std::endl;
  // std::cout << side << " alpha2:" << alpha2 << std::endl;
  // std::cout << side << " alpha3:" << alpha3 << std::endl;
  // std::cout << side << " beta1:" << beta1 << std::endl;
  // std::cout << side << " beta3:" << beta3 << std::endl;
  // std::cout << side << " theta1:" << theta1 << std::endl;
  // std::cout << side << " theta2:" << theta2 << std::endl;
  // std::cout << side << " gamma1:" << gamma1 << std::endl;
  // std::cout << side << " startingDifference:" << startingDifference << std::endl;
  // std::cout << side << " gammcurrentDifferencea1:" << currentDifference << std::endl;
  // std::cout << side << " tempTheta:" << tempTheta << std::endl;
  // std::cout << side << " templ2:" << templ2 << std::endl;

  return true;
}

bool thighFkine2(std::string side, std::map<std::string, geometry_msgs::TransformStamped>& transformMap) {
  // Kinematics for main leg mechanism including thighs knees shins
  // This version uses generic 4-bar linkage kinematics instead of over complex
  double lr1; //Distance between knees
  Relative_Distance_In_Tree(transformMap, side + "_shin_b_link", side + "_shin_a_link", "base_link", lr1);
  double l_knee_b_to_shin_connection = 0.070;
  double l_shin_a2 = 0.3206;
  double l_thigh_a2 = 0.100;
  double l_thigh_b2 = 0.300;
  double zero = 0;


  double alpha1 = angleCosineRule(l_knee_b_to_shin_connection, l_shin_a2, lr1); // bottom half of knee a angle
  double beta1 = angleCosineRule(l_thigh_b2, l_thigh_a2, lr1); // top half of knee a angle
  double theta1 = -(M_PI - (alpha1 + beta1));//Angle to set shin a to;

  double alpha2 = angleCosineRule(l_shin_a2, lr1, l_knee_b_to_shin_connection); // bottom half of knee b angle
  double beta2 = angleCosineRule(l_thigh_a2, lr1, l_thigh_b2); // top half of knee b angle
  double theta2 = M_PI - (alpha2 + beta2); //Angle to set shin b to;
  transformMap[side + "_shin_a_link"].transform.rotation = setRPY(zero, zero, theta1);
  transformMap[side + "_shin_b_link"].transform.rotation = setRPY(zero, zero, theta2);

  return true;
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
  //std::cout << "Local1: " << localX1 << ", " << localY1 << ", " << 0 << std::endl;
  //td::cout << "Local2: " << localX2 << ", " << localY2 << ", " << 0 << std::endl;
  /*eometry_msgs::TransformStamped resultb;
  geometry_msgs::TransformStamped resulta;
  Relative_TF_In_Chain(transforms, transforms[prefix + "_shin_link_b"].child_frame_id, "base_link", "base_link", resultb);
  Relative_TF_In_Chain(transforms, transforms[prefix + "_shin_link_a"].child_frame_id, "base_link", "base_link", resulta);
  std::cout << "Relative a: " << resulta.transform.translation.x << ", " << resulta.transform.translation.y << ", " << resulta.transform.translation.z << std::endl;
  std::cout << "Relative b: " << resultb.transform.translation.x << ", " << resultb.transform.translation.y << ", " << resultb.transform.translation.z << std::endl;
  std::cout << "Relative LR1:  " << distance(resulta.transform.translation.x, resulta.transform.translation.y, resulta.transform.translation.z, resultb.transform.translation.x, resultb.transform.translation.y, resultb.transform.translation.z) << std::endl;
*/
  double lr1 = distance(localX1, localY1, localX2, localY2);
  std::cout << "LR1: " << lr1 << std::endl;
  //geometry_msgs::TransformStamped test = getOffsetTF(transforms[prefix + "_shin_link_b"], transforms[prefix + "_shin_link_a"], transforms);
  //std::cout << "LR1 using func: " << distance(test.transform.translation.x, test.transform.translation.y, test.transform.translation.z, 0, 0, 0) << std::endl;
  double lr1TestA;
  Relative_Distance_In_Tree(transforms, prefix + "_shin_link_b", prefix + "_shin_link_a", "base_link", lr1TestA);
  std::cout << "LR1a vs LR1TestA: " << lr1 << " vs " << lr1TestA << std::endl;


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
  double lr1Test;
  if (Relative_Distance_In_Tree(transforms, prefix + "_ankle_link_b", prefix + "_foot_link_b", prefix + "_knee_link_b", lr1Test)) {
    std::cout << "First true" << std::endl;
  } else {
    std::cout << "First false" << std::endl;
  }

  geometry_msgs::TransformStamped test11 = getOffsetTF(transforms.at(prefix + "_ankle_link_b"), transforms.at(prefix + "_ankle_link_a"), transforms);
  std::cout << "Test11: " << distance(test11.transform.translation.x, test11.transform.translation.y, test11.transform.translation.z, 0, 0, 0) << std::endl;
  geometry_msgs::TransformStamped test12 = getOffsetTF(transforms.at(prefix + "_foot_link_b"), transforms.at(prefix + "_shin_link_connection"), transforms);
  std::cout << "Test12: " << distance(test12.transform.translation.x, test12.transform.translation.y, test12.transform.translation.z, 0, 0, 0) << std::endl;
  std::cout << "LR1b vs LR1Test: " << lr1 << " vs " << lr1Test << std::endl;
  double lr2 = distance(left_foot_offset.transform.translation.x, left_foot_offset.transform.translation.y, ankle_c_offset.transform.translation.x, ankle_c_offset.transform.translation.y);
  double lr2Test;

  if (Relative_Distance_In_Tree(transforms, prefix + "_ankle_link_c_1", prefix + "_foot_link_b", "base_link", lr2Test)) {
    std::cout << "First true" << std::endl;
  } else {
    std::cout << "First false" << std::endl;

  }
  std::cout << "LR2 vs LR2Test: " << lr2 << " vs " << lr2Test << std::endl;
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

  geometry_msgs::TransformStamped globalTF;
  if (Relative_TF_In_Chain(transforms, prefix + "_foot_link_a", "base_link", "base_link", globalTF)) {
    std::cout << prefix << " global position: " << globalTF.transform.translation.x << ", " << globalTF.transform.translation.y << ", " <<  globalTF.transform.translation.z << std::endl;
  }

  return false;
}

bool fkine(alex_kinematics::alex_fkine::Request &req, alex_kinematics::alex_fkine::Response &res) {
  std::vector<geometry_msgs::TransformStamped> tf_transforms_in = req.transforms;
  std::vector<geometry_msgs::TransformStamped> tf_transforms_out;
  std::map<std::string, geometry_msgs::TransformStamped> transformMap;

  for (auto x : tf_transforms_in) {
    transformMap[x.child_frame_id] = x;
  }

  hipFkine2("left", transformMap);
  thighFkine2("left", transformMap);

  hipFkine2("right", transformMap);
  thighFkine2("right", transformMap);
  //legFkine("right", transformMap);

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
