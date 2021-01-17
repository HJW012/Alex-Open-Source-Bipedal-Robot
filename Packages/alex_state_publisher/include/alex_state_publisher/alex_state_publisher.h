#ifndef ALEX_STATE_PUBLISHER_H
#define ALEX_STATE_PUBLISHER_H

#include <map>
#include <string>

#include <ros/ros.h>
#include <urdf/model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

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

protected:
  virtual void addChildren(const KDL::SegmentMap::const_iterator segment);

  std::map<std::string, SegmentPair> segments_, segments_fixed_;
  urdf::Model model_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

}

#endif
