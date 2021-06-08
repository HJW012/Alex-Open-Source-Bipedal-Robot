#include <alex_kinematics/alex_fkine_node.h>

bool getKineParams() {

  ros::param::get("/legs/hipSigma/S0", hipSigma0);
  ros::param::get("/legs/hipSigma/S1", hipSigma1);
  ros::param::get("/legs/hipSigma/S2", hipSigma2);
  ros::param::get("/legs/hipSigma/S3", hipSigma3);
  ros::param::get("/legs/hipSigma/S4", hipSigma4);
  ros::param::get("/legs/hipSigma/S5", hipSigma5);
  ros::param::get("/legs/hipSigma/S6", hipSigma6);
  ros::param::get("/legs/hipSigma/S7", hipSigma7);
  ros::param::get("/legs/hipSigma/S8", hipSigma8);
  hipSigma0 = deg2rad(hipSigma0);
  hipSigma1 = deg2rad(hipSigma1);
  hipSigma2 = deg2rad(hipSigma2);
  hipSigma3 = deg2rad(hipSigma3);
  hipSigma4 = deg2rad(hipSigma4);
  hipSigma5 = deg2rad(hipSigma5);
  hipSigma6 = deg2rad(hipSigma6);
  hipSigma7 = deg2rad(hipSigma7);
  hipSigma8 = deg2rad(hipSigma8);

  ros::param::get("/legs/hipLengths/L1", hipL1);
  ros::param::get("/legs/hipLengths/L2", hipL2);
  ros::param::get("/legs/hipLengths/L3", hipL3);
  ros::param::get("/legs/hipLengths/L4", hipL4);
  ros::param::get("/legs/hipLengths/L5", hipL5);
  ros::param::get("/legs/hipLengths/L6", hipL6);
  ros::param::get("/legs/hipLengths/L7", hipL7);

   ros::param::get("/system/UseCADLengths", useCADLengths);
   if (useCADLengths) {
     ros::param::get("/legs/lowerLegLengths_CAD/L1", lowerLegL1);
     ros::param::get("/legs/lowerLegLengths_CAD/L2", lowerLegL2);
     ros::param::get("/legs/lowerLegLengths_CAD/L3", lowerLegL3);
     ros::param::get("/legs/lowerLegLengths_CAD/L4", lowerLegL4);
     ros::param::get("/legs/lowerLegLengths_CAD/L5", lowerLegL5);
   } else {
     ros::param::get("/legs/lowerLegLengths_Actual/L1", lowerLegL1);
     ros::param::get("/legs/lowerLegLengths_Actual/L2", lowerLegL2);
     ros::param::get("/legs/lowerLegLengths_Actual/L3", lowerLegL3);
     ros::param::get("/legs/lowerLegLengths_Actual/L4", lowerLegL4);
     ros::param::get("/legs/lowerLegLengths_Actual/L5", lowerLegL5);
   }
   return true;
}

bool hipFkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  int side;
  if (prefix == "left") {
    side = -1;
  } else if (prefix == "right") {
    side = 1;
  }

  double roll, pitch, yaw;
  getRPY(transforms[prefix + "_hip_p1_to_hip_p2"].transform.rotation, roll, pitch, yaw);

  double lr0, lr1;
  if (!Relative_Distance_In_Tree(transforms, prefix + "_hip_p1_to_hip_p2", prefix + "_hip_p4_to_hip_p5", "base_link", lr0)) {
    return false;
  }

  double gamma1 = hipSigma7 + yaw;
  if (!Relative_Distance_In_Tree(transforms, prefix + "_hip_p2_to_hip_p3", prefix + "_hip_p4_to_hip_p5", "base_link", lr1)) {
    return false;
  }
  double alpha1 = angleCosineRule(lr0, hipL3, lr1);
  double alpha2 = angleCosineRule(hipL5, hipL4, lr1);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(hipL3, lr1, lr0);
  double beta2 = angleCosineRule(hipL4, hipL5, lr1);
  double beta = beta1 + beta2;

  getRPY(transforms[prefix + "_hip_p2_to_hip_p3"].transform.rotation, roll, pitch, yaw);
  yaw = -(M_PI - alpha);
  transforms[prefix + "_hip_p2_to_hip_p3"].transform.rotation = setRPY(roll, pitch, yaw);

  double gamma2 = angleCosineRule(lr1, hipL4, hipL5);
  getRPY(transforms[prefix + "_hip_p3_to_hip_p4"].transform.rotation, roll, pitch, yaw);
  yaw = -(M_PI - gamma2);
  transforms[prefix + "_hip_p3_to_hip_p4"].transform.rotation = setRPY(roll, pitch, yaw);

  double swayAngle = 2 * M_PI - (hipSigma3 + hipSigma6 + hipSigma8 + beta);
  getRPY(transforms[prefix + "_hip_p4_to_hip_p5"].transform.rotation, roll, pitch, yaw);
  yaw = ((swayAngle + hipSigma8) - M_PI);
  transforms[prefix + "_hip_p4_to_hip_p5"].transform.rotation = setRPY(roll, pitch, yaw);

  return true;
}

bool lowerLegFkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  int side;
  if (prefix == "left") {
    side = -1;
  } else if (prefix == "right") {
    side = 1;
  }

  double lr1;
  if (!Relative_Distance_In_Tree(transforms, prefix + "_main_p2_to_main_p4", prefix + "_main_p1_to_main_p3", "base_link", lr1)) {
    return false;
  }

  double alpha1 = angleCosineRule(lowerLegL1, lowerLegL2, lr1);
  double alpha2 = angleCosineRule(lowerLegL3, lowerLegL4, lr1);
  double alpha = alpha1 + alpha2;

  double gamma1 = angleCosineRule(lr1, lowerLegL2, lowerLegL1);
  double gamma2 = angleCosineRule(lr1, lowerLegL3, lowerLegL4);

  double beta1 = angleCosineRule(lowerLegL2, lowerLegL1, lr1);
  double beta2 = angleCosineRule(lowerLegL4, lowerLegL3, lr1);
  double beta = beta1 + beta2;

  double roll, pitch, yaw;
  getRPY(transforms[prefix + "_main_p2_to_main_p4"].transform.rotation, roll, pitch, yaw);
  yaw = (M_PI - alpha);
  transforms[prefix + "_main_p2_to_main_p4"].transform.rotation = setRPY(roll, pitch, yaw);

  getRPY(transforms[prefix + "_main_p1_to_main_p3"].transform.rotation, roll, pitch, yaw);
  yaw = -(M_PI - beta);
  transforms[prefix + "_main_p1_to_main_p3"].transform.rotation = setRPY(roll, pitch, yaw);

  return true;
}

bool legFkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  hipFkine(prefix, transforms);
  lowerLegFkine(prefix, transforms);

  return true;
}

bool fkine(alex_kinematics::alex_fkine::Request &req, alex_kinematics::alex_fkine::Response &res) {
  std::vector<geometry_msgs::TransformStamped> transformsOut;
  std::map<std::string, geometry_msgs::TransformStamped> transforms;
  for (auto x : req.transforms) {
    transforms[x.child_frame_id] = x;
  }


  legFkine("left", transforms);
  legFkine("right", transforms);


  for (std::map<std::string, geometry_msgs::TransformStamped>::iterator i = transforms.begin(); i != transforms.end(); i++) {
     transformsOut.push_back(i->second);
   }
  res.transforms = transformsOut;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_fkine_node");
  ros::NodeHandle n;
  getKineParams();
  ros::ServiceServer service = n.advertiseService("alex_fkine", fkine);
  ROS_INFO("Alex Fkine Node");

  ros::spin();

  return 0;
}
