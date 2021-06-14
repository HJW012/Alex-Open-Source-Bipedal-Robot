#ifndef AlexLib_h
#define AlexLib_h

#include "Arduino.h"
#include "FlexCAN_T4.h"
#include "Dynamixel2Arduino.h"

#define DXL_DIR_PIN 10 // DYNAMIXEL Shield DIR PIN
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
#define DEBUG(x) DEBUG_SERIAL.println(x)
#define TMotor_send_delay 300 //Microseconds
#define jointAllowance 0.025f
#define poseAllowance 2.0
#define maxPoseX 180.0f
#define minPoseX -170.0f
#define minPoseZ -600
#define maxPoseZ -480
#define minPoseY 30
#define maxPoseY 150

#define setKP 50
#define setKD 1
//#define deg2rad M_PI/180
//#define rad2deg 180/M_PI

using namespace ControlTableItem;

// Generic Functions
class coord {
	private:
	public:
	double x = 0;
	double y = 0;
	double z = 0;
	
	coord(double, double, double);
	coord();
};

double deg2rad(double);
double rad2deg(double);
double distance(coord, coord);
double angleCosineRule(double, double, double);
double sideCosineRule(double, double, double);

//DYNAMIXEL
struct Dynamixel_status {
  bool inTorqueMode;
  bool running = false;
  bool ledOn = false;
  double lastPose = 0;
  long long int last_status_timestamp = 0;
  bool connected = false;
};

struct Dynamixel_properties {
	// All in degrees - min and max should swap for right leg and all motion should be reversed
  double minPose = 155;
  double maxPose = 220;
  double zeroPose = 0; // 2048 in step count
  int dirPin = 10;
};

struct Dynamixel_in {
  bool zeroMotor = false;
  bool toggleTorqueMode = false;
  double p_in = 0;
  double v_in = 0;
  double t_in = 0;
};

struct Dynamixel_out {
  double p_out = 0;
  double v_out = 0;
  double t_out = 0;
  
  double converted_p_out = 0; // These values are the result of manipulating p_out to get angle in right direction and incorporating the zero location
  double converted_v_out = 0;
  double converted_t_out = 0;
};

class Dynamixel {
	private:
	
	public:
	byte ID;
    byte secondary_ID;
    Dynamixel_properties properties;
    Dynamixel_status status;
    Dynamixel_in in;
    Dynamixel_out out;
	Dynamixel2Arduino dxl = new Dynamixel2Arduino(DXL_SERIAL, DXL_DIR_PIN); 
	
	Dynamixel();
	void init();
	void zero();
	void enterTorqueMode();
	void exitTorqueMode();
	void home();
	void sendReceiveCycle();
	void updateStatus();
	void convertOutputs(); // Converts status values from motor to suit kinematics
};

//TMOTOR
struct TMotor_status {
  bool inMotorMode = false;
  bool running = false;
  double lastPose = 0;
  long long int last_status_timestamp = 0;
  bool connected = false;
};

struct TMotor_properties {
  double P_MIN = -12.5f;
  double P_MAX = 12.5f;
  double V_MIN = -45.0f;
  double V_MAX = 45.0;
  double KP_MIN = 0.0f;
  double KP_MAX = 500.0f;
  double KD_MIN = 0.0f;
  double KD_MAX = 5.0f;
  double T_MIN = -18.0f;
  double T_MAX = 18.0f;

  double minPose = 0;
  double maxPose = 0;
  
  double zeroPose = 0;
};

struct TMotor_in { // In to the motor
  double p_in = 0;
  double v_in = 0;
  double kp_in = 0;
  double kd_in = 0;
  double t_in = 0;
  bool zeroMotor = false;
  bool toggleMotorMode = false;
  CAN_message_t statusMsg;
};

struct TMotor_out { // Out from the motor
  double p_out = 0;
  double v_out = 0;
  double t_out = 0;
  
  double converted_p_out = 0; // These values are the result of manipulating p_out to get angle in right direction and incorporating the zero location
  double converted_v_out = 0;
  double converted_t_out = 0;
  CAN_message_t commandMsg;
};

class TMotor {
	private:
	public:
	byte ID;
    TMotor_properties properties;
    TMotor_status status;
    TMotor_in in;
    TMotor_out out;
	FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
	
	TMotor();
	void init();
	void zero();
	void enterMotorMode();
	void exitMotorMode();
	void pack_cmd();
	void unpack_reply();
	unsigned int float_to_uint(float, float, float, int);
	float uint_to_float(unsigned int, float, float, int);
	void sendReceiveCycle();
	void updateStatus();
	void convertOutputs(); // Converts status values from motor to suit kinematics
};

//LEG

struct Leg_properties {
  bool side = 0; // 0 for left, 1 for right

  coord maxFootprint[]; // Outline footprint which food cant go outside of - needs to be set
  coord hipP[10]; // Each point location in the leg - relative based off of P0
  coord mainP[10];
  // From hip mechanism Fkine calcs:
  //  l1, l2, l3, l4, l5, l6, l7
  //  s0, s1, s2, s3, s4, s5, s6, s7, s8
  double hipLengths[7] = {59.96, 33.8, 46.803, 43.656, 67.801, 68.139, 59.561};
  double hipSigma[9] = {deg2rad(149.22), deg2rad(134.896), deg2rad(75.884), deg2rad(109.314), deg2rad(149.22), deg2rad(75.425), deg2rad(28.691), deg2rad(104.574), deg2rad(75.884)};

  // From main mechanism Fkine calcs:
  //  l1, l2, l3, l4, l5
  //double mainLengths[5] = {100, 296.102, 320.6, 70, 205}; // CAD lengths
  double mainLengths[5] = {100, 310, 315, 105, 170}; // Actual lengths

  double swayAngle = 0; // Calculated angle of lower leg to horizontal as viewed form the back (iota on fkine calc)

  double deltaX = 0;
  double deltaZ = 0;

  // Current joint angles based on statusses from motors
  double current_q0 = 0; // hip
  double current_q1 = 0; // long link 1
  double current_q2 = 0; // short link 2'
  coord current_foot_pose;
  
  // Calcualted joint angles from kinematics
  double calculated_q0 = 0;
  double calculated_q1 = 0;
  double calculated_q2 = 0;
  coord calculated_foot_pose;
  
  double goal_q0 = 0;
  double goal_q1 = 0;
  double goal_q2 = 0;
  coord goal_foot_pose;
  
  bool trackGoalFootPose = false;

  coord fkineFootLocation_virtual;
  coord fkineFootLocation_actual;
  coord presentFootPose;
};

struct Leg_status {
  bool running = false;
  coord lastFootPose;
  bool inMotorMode = false;
};

struct Leg_in {

};

struct Leg_out {

};

class Leg {
	private:
	public:
	Dynamixel hip_motor;
	TMotor knee_motor_1;
	TMotor knee_motor_2;
	Leg_properties properties;
	Leg_status status;
	Leg_in in;
	Leg_out out;
	Dynamixel2Arduino dxl = new Dynamixel2Arduino(DXL_SERIAL, DXL_DIR_PIN); 
	
	Leg();
	void init();
	void hipFkine(double);
	void mainFkine(double, double);
	void fkine(double, double, double);
	void hipIkine(coord);
	void mainIkine(coord);
	void ikine(coord);
	void zeroMotors();
	void sendReceiveCycle();
	void updateJointStatus();
	void updatePoseStatus();
	void toggleGoalTracking(bool);
	void toggleGoalTracking();
	void convertJointAngles();
	void motorsOff();
	void motorsOn();
	void moveToJointAngles();
	void moveToPose();
	void convertOutputs();
};

//ALEX

struct Alex_properties {
  const float dynamixel_protocol_version = 2.0;
  bool trackGoalFootPose = false;

  // P0 - robot centre, P1 - left hip joint, P2 - right hip joint
  coord p[3];
};

struct Alex_status {
  bool running = false;
  coord lastBodyPose;
  bool fs = true;
  bool powerStatus = false;
  bool inMotorMode = false;
};

struct Alex_out {
	CAN_message_t commandMsg;
};

struct Alex_in {
	CAN_message_t statusMsg;
};

class Alex {
	private:
	public:
	Leg leg[2]; // 2 legs to allow leg 0 and leg 1 to be left and right
	Alex_properties properties;
	Alex_status status;
	Alex_out out;
	Alex_in in;
	FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
	Dynamixel2Arduino dxl = new Dynamixel2Arduino(DXL_SERIAL, DXL_DIR_PIN); 
	bool ES_Healthy = false;
	
	Alex();
	void init();
	void zeroMotors();
	void unpack_TMotor_reply();
	void getInitalPose();
	void sendReceiveCycle();
	void updateJointStatus();
	void updatePoseStatus();
	void convertJointAngles();
	void toggleGoalTracking();
	void calcJointAngles();
	void ES_check();
	void motorsOn();
	void motorsOff();
	void poseCheck();
};

class Alex_cycle {
	// 0: Receive Status from motors
	// 1: Update internal parameters
	// 2: Receive commands from external
	// 3: Check commands and manipulate internals
	// 4: Move
	
	private:
	public:
	int stepNo = 0;
	
	Alex_cycle();
	void step_0();
	void step_1();
	void step_2();
	void step_3();
	void step_4();
	
	
};

#endif