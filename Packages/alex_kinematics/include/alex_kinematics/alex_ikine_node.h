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

double hipL1, hipL2, hipL3, hipL4, hipL5, hipL6, hipL7;
double hipSigma0, hipSigma1, hipSigma2, hipSigma3, hipSigma4, hipSigma5, hipSigma6, hipSigma7, hipSigma8;
double lowerLegL1, lowerLegL2, lowerLegL3, lowerLegL4, lowerLegL5;
bool useCADLengths = false;


bool getKineParams();
