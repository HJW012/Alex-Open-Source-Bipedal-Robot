#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

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

// Generic Function Definitions
void test();
