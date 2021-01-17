#ifndef JOINT_STATE_LISTENER_H
#define JOINT_STATE_LISTENER_H

#include <memory>
#include <map>
#include <string>

#include <boost/scoped_ptr.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "alex_state_publisher/alex_state_publisher.h"

namespace alex_state_publisher {

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;
typedef std::map<std::string, urdf::JointMimicSharedPtr > MimicMap;

class JointStateListener {
public:
  /** Default constructor.
   */
  JointStateListener();

  /** Constructor
   * \param tree The kinematic model of a robot, represented by a KDL Tree
   */
  JointStateListener(const KDL::Tree& tree, const MimicMap& m, const urdf::Model& model = urdf::Model());

  JointStateListener(const std::shared_ptr<AlexStatePublisher>& rsp, const MimicMap& m);


  /// Destructor
  ~JointStateListener();

protected:
  virtual void callbackJointState(const JointStateConstPtr& state);
  virtual void callbackFixedJoint(const ros::TimerEvent& e);

  ros::Duration publish_interval_;
  std::shared_ptr<AlexStatePublisher> state_publisher_;
  ros::Subscriber joint_state_sub_;
  ros::Timer timer_;
  ros::Time last_callback_time_;
  std::map<std::string, ros::Time> last_publish_time_;
  MimicMap mimic_;
  bool use_tf_static_;
  bool ignore_timestamp_;

};
}

#endif
