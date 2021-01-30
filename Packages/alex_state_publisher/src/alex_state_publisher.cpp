#include <map>
#include <string>

#include <kdl/frames_io.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>

#include "alex_state_publisher/alex_state_publisher.h"

namespace alex_state_publisher {

AlexStatePublisher::AlexStatePublisher() : AlexStatePublisher(KDL::Tree())
{
}

AlexStatePublisher::AlexStatePublisher(const KDL::Tree& tree, const urdf::Model& model)
  : model_(model)
{
  // walk the tree and add segments to segments_
  addChildren(tree.getRootSegment());
}

// add children to correct maps
void AlexStatePublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
{
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

std::string stripSlash(const std::string & in)
{
  if (in.size() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

// publish moving transforms
void AlexStatePublisher::publishTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time)
{
  ROS_DEBUG("Publishing transforms for moving joints");
  std::vector<geometry_msgs::TransformStamped> tf_transforms;

  // Perform forward kinematics based on base_to_link1 and base_to_link2
/*
  std::cout << 0 << std::endl;
  std::map<int, geometry_msgs::TransformStamped> pointMap; // Map to join transform with point number for kinematics
  std::map<std::string, double>::const_iterator iter = joint_positions.begin();
  geometry_msgs::TransformStamped tf_transform;
  for (int i = 0; i < 4; i++) {
    std::cout << i << std::endl;
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(iter->first);
    if (seg != segments_.end()) {
      tf_transform = tf2::kdlToTransform(seg->second.segment.pose(iter->second));
      tf_transform.header.frame_id = stripSlash(seg->second.root);
      tf_transform.child_frame_id = stripSlash(seg->second.tip);
      // NEED TO SET TRANSFORM TIMESTAMP AFTER KINEMATICS
      pointMap[i] = tf_transform;
    }
    iter++;
  }
  std::cout << 1 << std::endl;
  fkine(pointMap);
  std::cout << 2 << std::endl;
  int i = 0;
  for (std::map<std::string, double>::const_iterator jnt = joint_positions.begin(); jnt != joint_positions.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = pointMap[i];
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = stripSlash(seg->second.root);
      tf_transform.child_frame_id = stripSlash(seg->second.tip);
      tf_transforms.push_back(tf_transform);
      i++;
    }
  }

  std::cout << 3 <<  std::endl;
*/
  // loop over all joints

/*
  std::map<int, geometry_msgs::TransformStamped> pointMap;
  int pointIndex = 0;
  int pointCount = 0;
  for (std::map<std::string, double>::const_iterator jnt = joint_positions.begin(); jnt != joint_positions.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = stripSlash(seg->second.root);
      tf_transform.child_frame_id = stripSlash(seg->second.tip);


      pointMap[pointIndex] = tf_transform;
      pointIndex++;
      pointCount = pointIndex;
      // tf_transform.transform.translation.x = 1;
      // tf_transform.transform.translation.y = 1;
      // tf_transform.transform.translation.z = 0;


    }
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", jnt->first.c_str());
    }
  }

  fkine(pointMap);

  for (int i = 0; i < pointCount; i++) {
    tf_transforms.push_back(pointMap[i]);
  }
  tf_broadcaster_.sendTransform(tf_transforms);
*/
  std::vector<geometry_msgs::TransformStamped> transforms_in;

  for (std::map<std::string, double>::const_iterator jnt = joint_positions.begin(); jnt != joint_positions.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = stripSlash(seg->second.root);
      tf_transform.child_frame_id = stripSlash(seg->second.tip);
      if (tf_transform.header.frame_id == "base_link" && tf_transform.child_frame_id == "link1") {
        transforms_in.push_back(tf_transform);
      } else if (tf_transform.header.frame_id == "base_link" && tf_transform.child_frame_id == "link2") {
        transforms_in.push_back(tf_transform);
      }

    }
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", jnt->first.c_str());
    }
  }

  alex_kinematics::alex_leg_fkine fkineSrv;
  fkineSrv.request.jointAngles = transforms_in;
  fkineClient.call(fkineSrv);
  tf_transforms = fkineSrv.response.transforms;

  tf_broadcaster_.sendTransform(tf_transforms);


  // Frames are all frames except base_link in the order shown in rviz (currently 1,2,3,4,5,null)
  /*
  int i = 0;
  for (std::map<std::string, double>::const_iterator jnt = joint_positions.begin(); jnt != joint_positions.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = stripSlash(seg->second.root);
      tf_transform.child_frame_id = stripSlash(seg->second.tip);
      // tf_transform.transform.translation.x = 1;
      // tf_transform.transform.translation.y = 1;
      // tf_transform.transform.translation.z = 0;


      tf_transforms.push_back(tf_transform);
    }
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", jnt->first.c_str());
    }
  }
  tf_broadcaster_.sendTransform(tf_transforms);
  */
}

// publish fixed transforms
void AlexStatePublisher::publishFixedTransforms(bool use_tf_static)
{
  ROS_DEBUG("Publishing transforms for fixed joints");
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  geometry_msgs::TransformStamped tf_transform;

  // loop over all fixed segments
  for (std::map<std::string, SegmentPair>::const_iterator seg = segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
    geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
    tf_transform.header.stamp = ros::Time::now();
    if (!use_tf_static) {
      tf_transform.header.stamp += ros::Duration(0.5);
    }
    tf_transform.header.frame_id = stripSlash(seg->second.root);
    tf_transform.child_frame_id = stripSlash(seg->second.tip);
    tf_transforms.push_back(tf_transform);
  }
  if (use_tf_static) {
    static_tf_broadcaster_.sendTransform(tf_transforms);
  }
  else {
    tf_broadcaster_.sendTransform(tf_transforms);
  }
}

double AlexStatePublisher::distance(double x1, double y1, double x2, double y2) {
  double dist = abs(sqrt(pow((y1 - y2), 2) + pow(x1 - x2, 2)));
  return dist;
}

double AlexStatePublisher::angleCosineRule(double a, double b, double c) {
  double A = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
  return A;
}

double AlexStatePublisher::sideCosineRule(double b, double c, double A) {
  double a = sqrt(pow(b, 2) + pow(c, 2) - 2 * b * c * cos(A));
  return a;
}

bool AlexStatePublisher::fkine(std::map<int, geometry_msgs::TransformStamped>& pointMap) { //ALL OF THIS NEEDS TO BE OPTIMISED
  int j = 0;
  for (std::map<int, geometry_msgs::TransformStamped>::const_iterator i = pointMap.begin(); i != pointMap.end(); i++) {
    pointMap[j].transform.translation.z = 0;
    j++;
  }

  double roll, pitch, yaw;
  getRPY(pointMap[0].transform.rotation, roll, pitch, yaw);
  double q1 = yaw;
  double localX1 = l1 * cos(q1);
  double localY1 = -l1 * sin(q1);

  getRPY(pointMap[1].transform.rotation, roll, pitch, yaw);
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

  pointMap[2].transform.rotation = setRPY(roll, pitch, yaw);
  yaw = (alpha - M_PI);


  pointMap[3].transform.rotation = setRPY(roll, pitch, yaw);
  tfScalar zero = 0;
  pointMap[4].transform.rotation = setRPY(zero, zero, zero);

  return false; //CHECK FOR NANs
}

tf::Quaternion AlexStatePublisher::quatConversion(geometry_msgs::Quaternion q) {
  tf::Quaternion Q(q.x, q.y, q.z, q.w);
  return Q;
}

geometry_msgs::Quaternion AlexStatePublisher::quatConversion(tf::Quaternion q) {
  geometry_msgs::Quaternion Q;
  Q.x = q.x();
  Q.y = q.y();
  Q.z = q.z();
  Q.w = q.w();
  return Q;
}

geometry_msgs::Quaternion AlexStatePublisher::setRPY(tfScalar& roll, tfScalar& pitch, tfScalar& yaw) {
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return quatConversion(q);
}

void AlexStatePublisher::getRPY(tf::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

void AlexStatePublisher::getRPY(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf::Quaternion Q = quatConversion(q);
  getRPY(Q, roll, pitch, yaw);
}

}
