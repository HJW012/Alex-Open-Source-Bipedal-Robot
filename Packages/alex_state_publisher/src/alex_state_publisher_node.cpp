#include <map>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "alex_state_publisher/alex_state_publisher.h"
#include "alex_state_publisher/joint_state_listener.h"

void completeTree();

// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "alex_teleop_state_publisher");
  ros::NodeHandle node;

  // gets the location of the robot description on the parameter server
  urdf::Model model;
  urdf::Model model2;
  if (!model.initParam("robot_description"))
    return 1;

  if (!model2.initParam("robot_description2"))
    return 1;

  KDL::Tree tree;
  KDL::Tree tree1;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return 1;
  }


  if (!kdl_parser::treeFromUrdfModel(model2, tree1)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return 1;
  }

  if (!tree.addTree(tree1, "base_link")) {
    std::cout << "ISSUES HERE" << std::endl;
  }
  std::cout << "Number of segments: " << tree.getNrOfSegments() << std::endl;
  std::cout << "Number of joints: " << tree.getNrOfJoints() << std::endl;
  std::cout << "Number of segments: " << tree1.getNrOfSegments() << std::endl;
  std::cout << "Number of joints: " << tree1.getNrOfJoints() << std::endl;

  alex_state_publisher::MimicMap mimic;
  for (std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++) {
    if (i->second->mimic) {
      mimic.insert(std::make_pair(i->first, i->second->mimic));
    }
  }

  alex_state_publisher::JointStateListener state_publisher(tree, mimic, model);

  ros::spin();

  return 0;
}

void completeTree() {
  KDL::Chain chain1; //Chain containing link2, link4, link5
  KDL::Chain chain2; //Chain containing link1, link3

  KDL::Joint rz = KDL::Joint(KDL::Joint::RotZ);

  //Links 1 and 2 are already created in the URDF



}
