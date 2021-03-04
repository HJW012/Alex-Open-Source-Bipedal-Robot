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

// Limb Lengths
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

// Kinematic Functions
void fkine();
void legFkine();
void ikine();
void hipikine();
void kneeIkine();
void ankleIkine();
tf2::Quaternion rotAdd(tf2::Quaternion, tf2::Quaternion);
geometry_msgs::Quaternion rotAdd(geometry_msgs::Quaternion, geometry_msgs::Quaternion);
geometry_msgs::TransformStamped getOffsetTF(geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, std::map<std::string, geometry_msgs::TransformStamped>);
bool Relative_TF_In_Chain(std::map<std::string, geometry_msgs::TransformStamped>, std::string, std::string, std::string, geometry_msgs::TransformStamped&);
bool Relative_Distance_In_Tree(std::map<std::string, geometry_msgs::TransformStamped>, std::string, std::string, std::string, double &);

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

// CAN Functions
float constrain(float, float, float);
unsigned int float_to_uint(float, float, float, int);
float uint_to_float(unsigned int, float, float, int);
