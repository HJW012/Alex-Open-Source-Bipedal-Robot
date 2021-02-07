#ifndef ALEX_STATE_PUBLISHER_H
#define ALEX_STATE_PUBLISHER_H

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
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include "alex_kinematics/alex_fkine.h"
#include "alex_kinematics/alex_ikine.h"

ros::ServiceClient fkineClient;
ros::ServiceClient ikineClient;

namespace alex_state_publisher {

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
    segment(p_segment), root(p_root), tip(p_tip){}

  KDL::Segment segment;
  std::string root, tip;
};


class AlexStatePublisher
{
public:
  /** Default constructor.
   */
  AlexStatePublisher();

  /** Constructor
   * \param tree The kinematic model of a robot, represented by a KDL Tree
   */
  AlexStatePublisher(const KDL::Tree& tree, const urdf::Model& model = urdf::Model());

  /// Destructor
  ~AlexStatePublisher(){};

  /** Publish transforms to tf
   * \param joint_positions A map of joint names and joint positions.
   * \param time The time at which the joint positions were recorded
   */
  virtual void publishTransforms(const std::map<std::string, double>& joint_positions, const ros::Time& time);
  virtual void publishFixedTransforms(bool use_tf_static = false);

  double distance(double, double, double, double);
  double angleCosineRule(double, double, double);
  double sideCosineRule(double, double, double);
  geometry_msgs::Quaternion setRPY(tfScalar&, tfScalar&, tfScalar&);
  void getRPY(tf::Quaternion, double&, double&, double&);
  void getRPY(geometry_msgs::Quaternion, double&, double&, double&);
  tf::Quaternion quatConversion(geometry_msgs::Quaternion);
  geometry_msgs::Quaternion quatConversion(tf::Quaternion);
  bool fkine(std::map<std::string, geometry_msgs::TransformStamped>&);


protected:
  virtual void addChildren(const KDL::SegmentMap::const_iterator segment);

  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  urdf::Model model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}

#endif
