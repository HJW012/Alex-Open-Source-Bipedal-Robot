#include "ros/ros.h"
#include <map>
#include <string>

#include <urdf/model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
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
#include <urdf/model.h>
#include "alex_kinematics/alex_ikine.h"
#include "alex_global/global_definitions.h"
#include <tf2_kdl/tf2_kdl.h>
#include <sensor_msgs/JointState.h>

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
bool hipIkine2(std::string side, geometry_msgs::TransformStamped target, std::map<std::string, geometry_msgs::TransformStamped>& transformMap);
bool kneeIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms);
bool ankleIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms);
geometry_msgs::TransformStamped getRelativeTransform(geometry_msgs::TransformStamped targetTF, geometry_msgs::TransformStamped parentTF);
bool getURDFTree(std::map<std::string, geometry_msgs::TransformStamped>& modelTree);
void addChildren(const KDL::SegmentMap::const_iterator segment);
void mapTransforms(std::map<std::string, geometry_msgs::TransformStamped>&);
std::string stripSlash(const std::string &);
sensor_msgs::JointState jointStates;

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
    segment(p_segment), root(p_root), tip(p_tip){}

  KDL::Segment segment;
  std::string root, tip;
};

std::map<std::string, SegmentPair> segments_, segments_fixed_;
std::map<std::string, geometry_msgs::TransformStamped> modelTree;
urdf::Model model_;
std::map<std::string, geometry_msgs::TransformStamped> transformMap;






bool legIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms) {
  std::map<std::string, geometry_msgs::TransformStamped> mappedTransforms = transforms;
  //hipIkine(prefix, transforms, mappedTransforms);   // Find hip joiniftyXnt angle and map foot coords to calculate knee/ankle joint angles
  kneeIkine(prefix, transforms, mappedTransforms);
  //ankleIkine(prefix, transforms, mappedTransforms);
  // foot transforms are relative to hips
  return true;
}

//Ikine for hip joint angles and to map foot coords - must be performed before knee ikine.
bool hipIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms) {
  geometry_msgs::TransformStamped footA = transforms[prefix + "_foot_a"];
  double knee_offset_yz = sqrt(pow(transformMap[prefix + "_knee_link_b"].transform.translation.y, 2) + pow(transformMap[prefix + "_knee_link_b"].transform.translation.x, 2));
  double lr1 = distance(0, 0, footA.transform.translation.y, footA.transform.translation.z);
  double alpha1 = angleCosineRule(abs(transformMap[prefix + "_knee_link_b"].transform.translation.x), knee_offset_yz, abs(transformMap[prefix + "_knee_link_b"].transform.translation.y));
  double alpha2 = M_PI - (M_PI/2 + alpha1);
  double theta = alpha1 + M_PI/2;
  double alpha = asin(knee_offset_yz * (sin(theta)/lr1));
  double gamma = M_PI - (theta + alpha);
  double foot_offset = sin(gamma) * (lr1/sin(theta));
  double iota = angleCosineRule(abs(footA.transform.translation.y), abs(footA.transform.translation.z), abs(lr1));
  double sigma;
  double hipJointAngle;
  if (prefix == "left") {
    double ohm = abs(atan(footA.transform.translation.z/footA.transform.translation.y));
    if (footA.transform.translation.y <= -abs(transformMap[prefix + "_knee_link_b"].transform.translation.y)) { //target to the left on default foot (vertical foot)
      sigma = M_PI - ohm;
      //hipJointAngle = -(iota + gamma - (M_PI - theta));
      hipJointAngle = -(sigma + gamma - (M_PI-theta) - M_PI/2);
    } else { //target to the right of default (vertical) foot
      sigma = ohm; //Sigma is angle of lr1
      // hipJointAngle = -(-iota + gamma - (M_PI - theta));
      hipJointAngle = -(sigma + gamma - (M_PI-theta) - M_PI/2);
    }
  } else if (prefix == "right") {

  }

  // COMMENTED OUT FOR Testing
  //jointStates.name.push_back("base_link_to_" + prefix + "_hip_link");
  //jointStates.position.push_back(hipJointAngle);

  mappedTransforms = transforms;
  return true;
}

bool hipIkine2(std::string side, geometry_msgs::TransformStamped target, std::map<std::string, geometry_msgs::TransformStamped>& transformMap) {
  geometry_msgs::TransformStamped tempP4;
  geometry_msgs::TransformStamped tempP0;
  Relative_TF_In_Chain(transformMap, side + "_coronal_hip_pivot_to_sagittal_hip_pivot_link", "base_link", "base_link", tempP4);
  Relative_TF_In_Chain(transformMap, side + "_symbolic_hip_to_coronal_hip_rotor_link", "base_link", "base_link", tempP0);
  double lr1 = distance(target, tempP4);
  double o_coronal_hip_pivot_to_sagittal_hip_pivot_y = 0.002471; //known and needs to be a param
  double alpha2 = 89 * M_PI/180; //known and needs to be a param
  std::vector<double> l2Vec;

  quadraticFormula(1, -(2 * o_coronal_hip_pivot_to_sagittal_hip_pivot_y * cos(alpha2)), (o_coronal_hip_pivot_to_sagittal_hip_pivot_y - pow(lr1, 2)), l2Vec);

  double l2;
  if (l2Vec.at(0) < 0) {
    l2 = l2Vec.at(1);
  } else {
    l2 = l2Vec.at(0);
  }
  std::cout << "l2: " << l2 << std::endl;

  double sigma3 = 199.314 * M_PI/180;
  std::cout << "o_coronal_hip_pivot_to_sagittal_hip_pivot_y ^ 2: " << pow(o_coronal_hip_pivot_to_sagittal_hip_pivot_y, 2) << std::endl;
  std::cout << "TEST: " << angleCosineRule(0.383481, 0.0025, 0.386727) << std::endl;
  double alpha1;
  alpha1 = angleCosineRule(l2, lr1, o_coronal_hip_pivot_to_sagittal_hip_pivot_y);
  double beta1 = sigma3 - alpha1;
  double l3 = 0.067801;
  double lr3 = sideCosineRule(l3, lr1, beta1);
  double sigma1;
  if (target.transform.translation.y > tempP4.transform.translation.y) {
    sigma1 = abs(atan((tempP4.transform.translation.z - target.transform.translation.z)/(tempP4.transform.translation.y - target.transform.translation.y)));
  } else {
    sigma1 = M_PI - abs(atan((tempP4.transform.translation.z - target.transform.translation.z)/(tempP4.transform.translation.y - target.transform.translation.y)));
  }

  double beta3 = angleCosineRule(l3, lr3, lr1);
  geometry_msgs::TransformStamped tempP3;
  tempP3.transform.translation.y = target.transform.translation.y + lr3 * cos(sigma1 + beta3);
  tempP3.transform.translation.z = target.transform.translation.z - lr3 * sin(sigma1 + beta3);
  geometry_msgs::TransformStamped tempP1;
  Relative_TF_In_Chain(transformMap, side + "_coronal_hip_torque_arm_link", "base_link", "base_link", tempP1);

  double lr4 = distance(tempP3, tempP1);
  double l5 = 0.046803;
  double l4 = 0.043656;
  double omega1 = angleCosineRule(l4, l5, lr4);
  double lr5 = distance(tempP1, tempP4);
  double gamma1 = angleCosineRule(l3, lr4, lr5);
  double l6 = 0.0338;
  double l7 = 0.068139;
  double delta1 = angleCosineRule(l7, l6, lr5);
  double theta = (omega1 + gamma1 + delta1) - M_PI;
  std::cout << "Hip Servo Angle: " << theta << std::endl;
  jointStates.name.push_back("temp");
  // jointStates.name.push_back(prefix + "_knee_link_b_to_" + prefix + "_knee_link_a");
  jointStates.position.push_back(-(0.1));
  // jointStates.position.push_back(q1 - o_knee_a + q2_offset);
  std::cout << "lr1: " << lr1 << std::endl;
  std::cout << "sigma3: " << sigma3 << std::endl;
  std::cout << "alpha1: " << alpha1 << std::endl;
  std::cout << "beta1: " << beta1 << std::endl;
  std::cout << "lr3: " << lr3 << std::endl;
  std::cout << "sigma1: " << sigma1 << std::endl;
  std::cout << "beta3: " << beta3 << std::endl;
  std::cout << "l5: " << l5 << std::endl;
  std::cout << "l4: " << l4 << std::endl;
  std::cout << "omega1: " << omega1 << std::endl;
  std::cout << "lr4: " << lr4 << std::endl;
  std::cout << "lr5: " << lr5 << std::endl;
  std::cout << "gamma1: " << gamma1 << std::endl;
  std::cout << "delta1: " << delta1 << std::endl;
  std::cout << "theta: " << theta << std::endl;

  return true;
}

//Ikine for knee joint angles - must be performed after hip ikine with mapped foot coords
bool kneeIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms) {
// foot A transform is relative to HIP JOINT - needs to be mapped to avoid knee offset
  geometry_msgs::TransformStamped footA;
  footA.transform.translation.x = mappedTransforms[prefix + "_foot_a"].transform.translation.x + o_hip_to_knee_x;
  footA.transform.translation.z = mappedTransforms[prefix + "_foot_a"].transform.translation.z + o_hip_to_knee_z;
  double lr1 = distance(0, 0, footA.transform.translation.x, footA.transform.translation.z);
  double alpha = angleCosineRule(lr1, l_knee_b, l_shin_b);
  double lr2 = sideCosineRule(l_knee_b, l_shin_connection, alpha);
  double beta = angleCosineRule(lr2, l_knee_a, l_shin_a);
  double delta1 = angleCosineRule(l_knee_b, l_shin_connection, lr2);
  double delta2 = angleCosineRule(l_knee_a, lr2, l_shin_a);
  double gamma1 = angleCosineRule(l_shin_connection, l_knee_b, lr2);
  double gamma2 = angleCosineRule(l_shin_a, lr2, l_knee_a);
  double gamma = gamma1 + gamma2;
  double iota = angleCosineRule(l_shin_b, l_knee_b, lr1);
  double sigma;

  if (footA.transform.translation.x > 0) {
    sigma = -atan((footA.transform.translation.z) / (footA.transform.translation.x));
  } else {
    sigma = M_PI - atan((footA.transform.translation.z) / (footA.transform.translation.x));
  }

  double q2 = (sigma + iota);
  double q2_offset = o_knee_b - q2;
  double q1 = q2 - gamma;

  jointStates.name.push_back(prefix + "_hip_link_to_" + prefix + "_knee_link_b");
  jointStates.name.push_back(prefix + "_knee_link_b_to_" + prefix + "_knee_link_a");
  jointStates.position.push_back(-(o_knee_b - q2));
  jointStates.position.push_back(q1 - o_knee_a + q2_offset);
  /*std::cout << "lr1: " << lr1 << std::endl;
  std::cout << "alpha: " << alpha << std::endl;
  std::cout << "lr2: " << lr2 << std::endl;
  std::cout << "beta: " << beta << std::endl;
  std::cout << "delta1: " << delta1 << std::endl;
  std::cout << "delta1: " << delta1 << std::endl;
  std::cout << "gamma1: " << gamma1 << std::endl;
  std::cout << "gamma2: " << gamma2 << std::endl;
  std::cout << "gamma: " << gamma << std::endl;
  std::cout << "iota: " << iota << std::endl;
  std::cout << "sigma: " << sigma << std::endl;
  std::cout << "o_knee_b: " << o_knee_b * 180 / M_PI << " deg" << std::endl;
  std::cout << "q2 offset: " << q2_offset * 180 / M_PI << " deg" << std::endl;
  std::cout << "Knee A Angle: " << (sigma + iota - gamma) * 180 / M_PI << " deg" << std::endl;
  std::cout << "Knee B Angle: " << (sigma + iota) * 180 / M_PI << " deg" << std::endl;
  std::cout << "Q1: " << (q1) * 180 / M_PI << " deg" << std::endl;
  std::cout << "Q2: " << (q2) * 180 / M_PI << " deg" << std::endl;*/
  return true;
}

// Ikine for ankle - must be performed after hip and knee ikine with mapped foot coords
bool ankleIkine(std::string prefix, std::map<std::string, geometry_msgs::TransformStamped>& transforms, std::map<std::string, geometry_msgs::TransformStamped>& mappedTransforms) {
  return true;
}

bool getURDFTree(ros::NodeHandle& n, std::map<std::string, geometry_msgs::TransformStamped>& modelTree) {
  std::string stringModel;
  urdf::Model model;

  n.getParam("robot_description", stringModel);
  model.initString(stringModel);
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return 1;
  }

  addChildren(tree.getRootSegment());
  mapTransforms(transformMap);
  return true;
}

void addChildren(const KDL::SegmentMap::const_iterator segment) {
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (size_t i = 0; i < children.size(); ++i) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName()) && model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
        ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", root.c_str(), child.getName().c_str());
      }
      else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
    }
    else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

void mapTransforms(std::map<std::string, geometry_msgs::TransformStamped>& transformMap) {
  for (std::map<std::string, SegmentPair>::const_iterator seg_ = segments_.begin(); seg_ != segments_.end(); seg_++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(seg_->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0)); //Use 0 since we only want the tree for dimensions, not joint angles
      tf_transform.header.frame_id = stripSlash(seg->second.root);
      tf_transform.child_frame_id = stripSlash(seg->second.tip);

      transformMap[tf_transform.child_frame_id] = tf_transform;
    }
  }
}

std::string stripSlash(const std::string & in) {
  if (in.size() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

bool ikine(alex_kinematics::alex_ikine::Request &req, alex_kinematics::alex_ikine::Response &res) {
  std::cout << "noe" << std::endl;
  std::vector<geometry_msgs::TransformStamped> targets;
  for (auto x : req.footTransforms) {
    targets.push_back(x);
  }

  // legIkine("left", transforms);
  std::cout << "noe 2" << std::endl;
  std::cout << transformMap.size() << std::endl;
  std::string side = "left";

  hipIkine2("left", targets.at(0), transformMap);
  // hipIkine2(side, geometry_msgs::TransformStamped target, std::map<std::string, geometry_msgs::TransformStamped>& transformMap);
  //legIkine("right", transforms);
  //res.jointStates = jointStates;
  std::cout << "noe 3" << std::endl;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alex_ikine_node");
  ros::NodeHandle n;

  getURDFTree(n, modelTree);
  ros::ServiceServer service = n.advertiseService("alex_ikine", ikine);
  ROS_INFO("Alex Ikine Node");
  ros::spin();

  return 0;
}
