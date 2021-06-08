#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <map>
#include <typeinfo>

// Settings Variables
class motor {
public:
  bool Enabled = false;
  int ID = false;
  motor() {

  }
};

class leg {
public:
  bool Enabled = false;
  motor M1;
  motor M2;
  motor M3;

  leg() {

  }
};

class SystemSettings {
public:
  bool UseMPU6050 = false;
  bool UseORIENTUS = false;
  bool AverageIMU = false;
  int IMUID1 = 0;
  int IMUID2 = 0;
  int PublishFrequency = 30;
};

leg leftLeg;
leg rightLeg;
SystemSettings systemSettings;

// Limb Lengths
double l_base_to_sybolic_hip_yz;
#define l_knee_a 0.100f
#define l_knee_b 0.30002f
#define l_shin_a 0.309f
#define l_shin_b 0.305f
#define l_shin_connection 0.115f
#define l_ankle_connection 0.07475f
#define l_ankle_a 0.02735f
#define l_ankle_b 0.04032f
#define l_ankle_c1 0.1493f
#define l_ankle_c2 0.155795f
#define l_foot_a 0.055f
#define l_foot_b 0.055f

// Joint Offsets
#define o_base_to_hip_X 0.0f
#define o_base_to_hip_y -0.5f
#define o_base_to_hip_z -0.1f
#define o_hip_to_knee_x 0.1f
#define o_hip_to_knee_y 0.1f
#define o_hip_to_knee_z 0.1f

// Joint Starting Orientaitons
#define o_knee_a M_PI/4
#define o_knee_b 3*M_PI/4

// TMotor Config Params
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Dynamixel global_definitions
#define ADDR_OPERATION_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_LED 65
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_CURRENT 126
#define ADDR_MOVING 122
#define ADDR_PRESENT_TEMPERATURE 146
#define ADDR_GOAL_VELOCITY 104
#define ADDR_GOAL_CURRENT 102
#define ADDR_VELOCITY_I_GAIN 76
#define ADDR_VELOCITY_P_GAIN 78
#define ADDR_POSITION_D_GAIN 80
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_P_GAIN 84
#define ADDR_DRIVE_MODE 10
#define ADDR_SHUTDOWN 63
#define ADDR_TEMPERATURE_LIMIT 31


// Protocol Version
#define PROTOCOL_VERSION 2.0

// Motor settings
#define LEFT_ID 1
#define RIGHT_ID 2
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyUSB0"

// Kinematic Functions

tf2::Quaternion rotAdd(tf2::Quaternion, tf2::Quaternion);
geometry_msgs::Quaternion rotAdd(geometry_msgs::Quaternion, geometry_msgs::Quaternion);
geometry_msgs::TransformStamped getOffsetTF(geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, std::map<std::string, geometry_msgs::TransformStamped>);
bool Relative_TF_In_Chain(std::map<std::string, geometry_msgs::TransformStamped>, std::string, std::string, std::string, geometry_msgs::TransformStamped&);
bool Relative_Distance_In_Tree(std::map<std::string, geometry_msgs::TransformStamped>, std::string, std::string, std::string, double &);
bool Relative_Quaternion_In_Chain(std::map<std::string, geometry_msgs::TransformStamped>, std::string, std::string, std::string, geometry_msgs::Quaternion);

// Kinematic chain functions
class kinematicObject;
class kinematicSubChain;
class kinematicLoop;
class kinematicChain;

class kinematicObject {
private:
public:
  std::string parentName = "";
  std::string childName = "";
  geometry_msgs::TransformStamped point;
  kinematicObject() {

  }

  kinematicObject(geometry_msgs::TransformStamped _tf) {
    parentName = _tf.header.frame_id;
    childName = _tf.child_frame_id;
    point = _tf;
  }

  virtual bool calculateFkine(std::map<std::string, geometry_msgs::TransformStamped> & transformMap) {

  }
};

class kinematicSubChain : public kinematicObject {
private:
public:
  std::vector<kinematicObject> chain; //Parent, object
  std::vector<double> linkLength; // Distances between TFs
  kinematicSubChain() {

  }


  void addObject(std::string parentJoint, kinematicObject object) {
    //chain.at(parentJoint) = object;
  }

  bool calculateFkine(std::map<std::string, geometry_msgs::TransformStamped> & transformMap) {
    for (auto x : chain) {
    /*  if (typeid(x.second) == typeid(kinematicLoop)) { //Maybe change this to typeof
        x.second.calculateFkine(transformMap);
      }*/
    }
  }
};

class kinematicLoop : public kinematicObject {
  // Convention:
  // - Known points should be their own chain - this chain should be element 0 in chains vector
  // - All other points should be other chain - this chain should be element 1 in chains vector
  // -


private:
  std::vector<std::string> transforms;
  std::vector<bool> controlledJoints; // Joints that are probably a motor - the child -- PROBABLY DONT NEED THIS

  std::vector<bool> knownJoints; // Joints whose transform can be used directly from the transformMap - should MUST be 3 joints for 4-bar linkages - two static and one motor end

  std::map<std::string, bool> loopJoints;
  std::map<std::string, bool> connections; // vector size should be number of joints-1 representing which joints are connected

  std::vector<kinematicSubChain> chains; // 0 should be knowns - 1 should be unknowns. 0 should be 3 joints
  std::map<std::string, geometry_msgs::TransformStamped> transformMap; // Entire TF map should be passed in
public:
  kinematicLoop() {

  }

  kinematicLoop(std::map<std::string, bool> loop) {
    loopJoints = loop;
  }

  void setLoop(std::map<std::string, bool> loop) {
    loopJoints = loop;
  }

  bool calculateFkine(std::map<std::string, geometry_msgs::TransformStamped> & transformMap) {
    int jointNumber = loopJoints.size();
    if (jointNumber == 3) {
      // 3 bar linkage
    } else if (jointNumber == 4) {
      //4 bar linkage

      // 1. find lr1 between non-connected joints
      geometry_msgs::TransformStamped point1;
      geometry_msgs::TransformStamped point2;
      bool lr1PointsFound = false;
      std::map<std::string, geometry_msgs::TransformStamped> tempMap;
      /*for (int i = 1; i < chains.at(0).chain.size(); i++) {
        if (chains.at(0).chain.at(i-1).point.header.frame_id != chains.at(0).chain.at(i).point.child_frame_id && chains.at(0).chain.at(i-1).point.child_frame_id != chains.at(0).chain.at(i).point.header.frame_id) { //If previous frame isnt the parent or child of current frame
          point1 = chains.at(0).chain.at(i-1).point;
          point2 = chains.at(0).chain.at(i).point;
          lr1PointsFound = true;
          break;
        }
      }
      if (!lr1PointsFound) {
        // didnt find two points that arent direct children/parents
        return false;
      }*/

      //for now assume that chain is in order along with lengths
      point1 = chains.at(0).chain.at(0).point;
      point2 = chains.at(0).chain.at(chains.at(0).chain.size()-1).point;
      double lr1 ;
      if (!Relative_Distance_In_Tree(transformMap, point1.child_frame_id, point2.child_frame_id, "base_link", lr1)) {
        // couldnt find relative distance in tree between two points
        return false;
      }

      point1 = chains.at(0).chain.at(0).point;
      point2 = chains.at(0).chain.at(chains.at(0).chain.size()-1).point;
      if (!Relative_Distance_In_Tree(transformMap, point1.child_frame_id, point2.child_frame_id, "base_link", lr1)) {
        // couldnt find relative distance in tree between two points
        return false;
      }



    } else if (jointNumber == 5) {
      //5 bar linkage
    }
  }
};

class kinematicChain : public kinematicObject {
private:
public:
  std::vector<kinematicObject> chain; //Parent, object
  kinematicChain() {

  }

  void addLoop(std::string parentJoint, kinematicLoop loop) {
    //chain.at(parentJoint) = loop;
  }

  void addObject(std::string parentJoint, kinematicObject object) {
    //chain.at(parentJoint) = object;
  }

  bool calculateFkine(std::map<std::string, geometry_msgs::TransformStamped> & transformMap) {
    for (auto x : chain) {
    /*  if (typeid(x.second) == typeid(kinematicLoop)) { //Maybe change this to typeof
        x.second.calculateFkine(transformMap);
      }*/
    }
  }
};

// General Functions
double distance(double, double, double, double);
double distance(double, double, double, double, double, double);
double distance(geometry_msgs::TransformStamped, geometry_msgs::TransformStamped);
double angleCosineRule(double, double, double);
double sideCosineRule(double, double, double);
tf2::Quaternion quatConversion(geometry_msgs::Quaternion);
geometry_msgs::Quaternion quatConversion(tf2::Quaternion);
tf2::Vector3 vector3Conversion(geometry_msgs::Vector3);
geometry_msgs::Vector3 vector3Conversion(tf2::Vector3);
geometry_msgs::Quaternion setRPY(tf2Scalar&, tf2Scalar&, tf2Scalar&);
void getRPY(tf2::Quaternion, double&, double&, double&);
void getRPY(geometry_msgs::Quaternion, double&, double&, double&);
bool getRosParams(ros::NodeHandle);
void quadraticFormula(double, double, double, std::vector<double>&);
double deg2rad(double);
double rad2deg(double);

// CAN Functions
float constrain(float, float, float);
unsigned int float_to_uint(float, float, float, int);
float uint_to_float(unsigned int, float, float, int);
