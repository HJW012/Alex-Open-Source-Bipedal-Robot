#include "Arduino.h"
#include "AlexLib.h"

// GENERIC
coord::coord() {

}

coord::coord(double x_, double y_, double z_) {
	x = x_;
	y = y_;
	z = z_;
}

double deg2rad(double ang) {
  return (ang * M_PI / 180);
}

double rad2deg(double ang) {
  return (ang * 180 / M_PI);
}

double distance(coord p1, coord p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

double angleCosineRule(double a, double b, double c) {
  double A = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
  return A;
}

double sideCosineRule(double b, double c, double A) {
  double a = sqrt(pow(b, 2) + pow(c, 2) - 2 * b * c * cos(A));
  return a;
}

// DYNAMIXEL
Dynamixel::Dynamixel() {
	
}

void Dynamixel::init() {
	dxl.ping(ID);
	exitTorqueMode();
	dxl.setOperatingMode(ID, OP_POSITION);
	dxl.writeControlTableItem(PROFILE_VELOCITY, ID, 30);
}

void Dynamixel::zero() {
	properties.zeroPose = out.p_out;
    delay(5);
}

void Dynamixel::enterTorqueMode() {
	dxl.ledOn(ID);
	dxl.torqueOn(ID);
}

void Dynamixel::exitTorqueMode() {
	dxl.torqueOff(ID);
	dxl.ledOff(ID);
}

void Dynamixel::home() {
	
}

void Dynamixel::sendReceiveCycle() {
	//Set position
	//Set time profile
	dxl.setGoalPosition(ID, in.p_in, UNIT_DEGREE);
	delayMicroseconds(TMotor_send_delay);
	out.v_out = dxl.getPresentVelocity(ID);
	out.p_out = dxl.getPresentPosition(ID, UNIT_DEGREE);
	
	// Get position
}

void Dynamixel::updateStatus() {
	
}

void Dynamixel::convertOutputs() {
	// Need to find a bertter method since this breaks once the raw output from the dxl goes negative
	//out.converted_p_out = fabs(out.p_out) - properties.zeroPose;
}

// TMOTOR
TMotor::TMotor() {
	
}

void TMotor::init() {
	out.commandMsg.len = 8;
	in.statusMsg.len = 8;
	out.commandMsg.id = ID;
	can1.begin();
	can1.setBaudRate(1000000);
}

void TMotor::zero() {
	out.commandMsg.id = ID;
	out.commandMsg.buf[0] = 0xFF;
	out.commandMsg.buf[1] = 0xFF;
	out.commandMsg.buf[2] = 0xFF;
	out.commandMsg.buf[3] = 0xFF;
	out.commandMsg.buf[4] = 0xFF;
	out.commandMsg.buf[5] = 0xFF;
	out.commandMsg.buf[6] = 0xFF;
	out.commandMsg.buf[7] = 0xFE;
	
	can1.write(out.commandMsg);
}

void TMotor::enterMotorMode() {
	out.commandMsg.id = ID;
	out.commandMsg.buf[0] = 0xFF;
	out.commandMsg.buf[1] = 0xFF;
	out.commandMsg.buf[2] = 0xFF;
	out.commandMsg.buf[3] = 0xFF;
	out.commandMsg.buf[4] = 0xFF;
	out.commandMsg.buf[5] = 0xFF;
	out.commandMsg.buf[6] = 0xFF;
	out.commandMsg.buf[7] = 0xFC;
	
	can1.write(out.commandMsg);
}

void TMotor::exitMotorMode() {
	out.commandMsg.id = ID;
	out.commandMsg.buf[0] = 0xFF;
	out.commandMsg.buf[1] = 0xFF;
	out.commandMsg.buf[2] = 0xFF;
	out.commandMsg.buf[3] = 0xFF;
	out.commandMsg.buf[4] = 0xFF;
	out.commandMsg.buf[5] = 0xFF;
	out.commandMsg.buf[6] = 0xFF;
	out.commandMsg.buf[7] = 0xFD;
	
	can1.write(out.commandMsg);
}

void TMotor::pack_cmd() {
  /// CAN COMMAND PACKET STRUCTURE ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, betweem -30 and 30 rad/s
  /// 12 bit kp, between 0 and 500 Nm/rad
  /// 12 bit kd, between 0 and 100 Nms/rad
  /// 12 bit feed forward torque, between -18 and 18 Nm
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows: For each quantity, bit 0 is Least Significant Bit
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity{11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]];
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  // Limit data to be within bounds
  float p_des = constrain(in.p_in, properties.P_MIN, properties.P_MAX);
  float v_des = constrain(in.v_in, properties.V_MIN, properties.V_MAX);
  float kp = constrain(in.kp_in, properties.KP_MIN, properties.KP_MAX);
  float kd = constrain(in.kd_in, properties.KD_MIN, properties.KD_MAX);
  float t_ff = constrain(in.t_in, properties.T_MIN, properties.T_MAX);

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, properties.P_MIN, properties.P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, properties.V_MIN, properties.V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, properties.KP_MIN, properties.KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, properties.KD_MIN, properties.KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, properties.T_MIN, properties.T_MAX, 12);

  // Pack ints into CAN buffer
  out.commandMsg.id = ID;
  out.commandMsg.buf[0] = p_int >> 8;
  out.commandMsg.buf[1] = p_int & 0xFF;
  out.commandMsg.buf[2] = v_int >> 4;
  out.commandMsg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  out.commandMsg.buf[4] = kp_int & 0xFF;
  out.commandMsg.buf[5] = kd_int >> 4;
  out.commandMsg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  out.commandMsg.buf[7] = t_int & 0xFF;
  
  can1.write(out.commandMsg);
}

void TMotor::unpack_reply() {
  /// CAN REPLY PACKET STRUCTURE
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and 30 rad/s
  /// 12 bit current, between -40 and 40
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows: For each quantity, bit 0 is Lease Significant Bit
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  byte buf[8];
  byte len = 0;
  if (can1.read(in.statusMsg)) {
    unsigned long canId = in.statusMsg.id;

    for (int i = 0; i < 8; i++) {
      buf[i] = in.statusMsg.buf[i];
    }

    // Unpack ints from CAN buffer
    unsigned int id = buf[0];
    if (id == ID) {
		unsigned int p_int = (buf[1] << 8) | buf[2];
		unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
		unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];

		// Convert unsigned ints to floats
		out.p_out = uint_to_float(p_int, properties.P_MIN, properties.P_MAX, 16);
		out.v_out = uint_to_float(v_int, properties.V_MIN, properties.V_MAX, 12);
		out.t_out = uint_to_float(i_int, -properties.T_MAX, properties.T_MAX, 12);
    } 
  }
}

unsigned int TMotor::float_to_uint(float x, float x_min, float x_max, int bits) {
	/// Converts a float to an unsigned int, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int) ((x - offset) * 4095.0 / span);
  }
  if (bits == 16) {
    pgg = (unsigned int) ((x - offset) * 65535.0 / span);
  }
  return pgg;
}

float TMotor::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
	/// Converts unsigned int to float, given range and number of
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float) x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float) x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

void TMotor::sendReceiveCycle() {
	pack_cmd();
	delayMicroseconds(TMotor_send_delay);
	unpack_reply();
}

void TMotor::updateStatus() {
	
}

void TMotor::convertOutputs() {
	//out.converted_p_out = M_PI - fabs(out.p_out);// - properties.zeroPose;
}

// LEG
Leg::Leg() {
	
}

void Leg::init() {
	knee_motor_1.init();
	knee_motor_2.init();
	hip_motor.dxl = dxl;
	hip_motor.init();
	
	knee_motor_1.properties.zeroPose = 3 * M_PI/4; // Radians
	knee_motor_2.properties.zeroPose = M_PI / 4;
	hip_motor.properties.zeroPose = 180;
	
	motorsOff();
}

void Leg::hipFkine(double q) {
  int side;
  if (properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  properties.hipP[1].y = properties.hipP[0].y + side * properties.hipLengths[0] * cos(properties.hipSigma[0]);
  properties.hipP[1].z = properties.hipP[0].z - properties.hipLengths[0] * sin(properties.hipSigma[0]);
  
  
  properties.hipP[2].y = properties.hipP[1].y + side * properties.hipLengths[1] * cos(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])));
  properties.hipP[2].z = properties.hipP[1].z - properties.hipLengths[1] * sin(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])));
  
  
  properties.hipP[5].y = properties.hipP[1].y + side * properties.hipLengths[5] * cos(properties.hipSigma[0] - (M_PI - properties.hipSigma[1]));
  properties.hipP[5].z = properties.hipP[1].z - properties.hipLengths[5] * sin(properties.hipSigma[0] - (M_PI - properties.hipSigma[1]));
  
  coord tempP2(0, 0, 0);
  tempP2.x = properties.hipP[2].y;
  tempP2.z = properties.hipP[2].z;
  coord tempP5(0, 0, 0);
  tempP5.x = properties.hipP[5].y;
  tempP5.z = properties.hipP[5].z;
  
  double lr0 = distance(properties.hipP[2], properties.hipP[5]);

  double gamma1 = properties.hipSigma[7] + q;

  properties.hipP[3].y = properties.hipP[2].y + side * properties.hipLengths[2] * cos(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])) - (M_PI - (properties.hipSigma[5] + gamma1)));
  properties.hipP[3].z = properties.hipP[2].z - properties.hipLengths[2] * sin(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])) - (M_PI - (properties.hipSigma[5] + gamma1)));

  double lr1 = distance(properties.hipP[3], properties.hipP[5]);

  double alpha1 = angleCosineRule(lr0, properties.hipLengths[2], lr1);
  double alpha2 = angleCosineRule(properties.hipLengths[4], properties.hipLengths[3], lr1);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(properties.hipLengths[2], lr1, lr0);
  double beta2 = angleCosineRule(properties.hipLengths[3], properties.hipLengths[4], lr1);
  double beta = beta1 + beta2;

  properties.hipP[4].y = properties.hipP[3].y + side * properties.hipLengths[3] * cos(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])) - (M_PI - (properties.hipSigma[5] + gamma1)) - (M_PI - alpha));
  properties.hipP[4].z = properties.hipP[3].z - properties.hipLengths[3] * sin(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])) - (M_PI - (properties.hipSigma[5] + gamma1)) - (M_PI - alpha));

  properties.swayAngle = 2 * M_PI - (properties.hipSigma[3] + properties.hipSigma[6] + properties.hipSigma[8] + beta);

  properties.hipP[6].y = properties.hipP[5].y + side * properties.hipLengths[6] * cos(properties.swayAngle);
  properties.hipP[6].z = properties.hipP[5].z - properties.hipLengths[6] * sin(properties.swayAngle);
 }

void Leg::mainFkine(double q1, double q2) {
  int side;
  if (properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  properties.mainP[0] = properties.hipP[6];

  properties.mainP[1].x = properties.mainP[0].x + properties.mainLengths[0] * cos(q2);
  properties.mainP[1].z = properties.mainP[0].z - properties.mainLengths[0] * sin(q2);

  properties.mainP[2].x = properties.mainP[0].x + properties.mainLengths[1] * cos(q1);
  properties.mainP[2].z = properties.mainP[0].z - properties.mainLengths[1] * sin(q1);
  
  coord tempP2(0, 0, 0);
  tempP2.x = properties.mainP[2].x;
  tempP2.z = properties.mainP[2].z;
  coord tempP1(0, 0, 0);
  tempP1.x = properties.mainP[1].x;
  tempP1.z = properties.mainP[1].z;

  double lr1 = distance(tempP2, tempP1);

  double alpha1 = angleCosineRule(properties.mainLengths[0], properties.mainLengths[1], lr1);
  double alpha2 = angleCosineRule(properties.mainLengths[2], lr1, properties.mainLengths[3]);
  double alpha = alpha1 + alpha2;

  double gamma1 = angleCosineRule(lr1, properties.mainLengths[1], properties.mainLengths[0]); // or q2 - q1
  double gamma2 = angleCosineRule(lr1, properties.mainLengths[3], properties.mainLengths[2]);

  double beta1 = angleCosineRule(properties.mainLengths[1], lr1, properties.mainLengths[0]); // or M_PI - (alpha1 + gamma1)
  double beta2 = angleCosineRule(properties.mainLengths[3], lr1, properties.mainLengths[2]); // or M_PI - (alpha2 + gamma2)
  double beta = beta1 + beta2;

  properties.mainP[3].x = properties.mainP[2].x + properties.mainLengths[3] * cos(q1 - (M_PI - alpha));
  properties.mainP[3].z = properties.mainP[2].z - properties.mainLengths[3] * sin(q1 - (M_PI - alpha));
  // COMPARE THESE TO AVOID IMPOSSIBLE JOINT ANGLES
  properties.mainP[3].x = properties.mainP[1].x + properties.mainLengths[2] * cos(q2 + (M_PI - beta));
  properties.mainP[3].z = properties.mainP[1].z - properties.mainLengths[2] * sin(q2 + (M_PI - beta));

  properties.mainP[4].x = properties.mainP[2].x + (properties.mainLengths[3] + properties.mainLengths[4]) * cos(q1 - (M_PI - alpha));
  properties.mainP[4].z = properties.mainP[2].z - (properties.mainLengths[3] + properties.mainLengths[4]) * sin(q1 - (M_PI - alpha));
  
  properties.deltaX = properties.mainP[4].x - properties.mainP[0].x;
  properties.deltaZ = properties.mainP[0].z - properties.mainP[4].z;

  //Calculate y of each point based on leg sway and update z of each point to accomodate this sway angle
  double deltaZ1 = properties.mainP[0].z - properties.mainP[1].z;
  properties.mainP[1].y = properties.mainP[0].y + side * deltaZ1 * cos(properties.swayAngle);
  properties.mainP[1].z = properties.mainP[0].z - deltaZ1 * sin(properties.swayAngle);

  double deltaZ2 = properties.mainP[0].z - properties.mainP[2].z;
  properties.mainP[2].y = properties.mainP[0].y + side * deltaZ2 * cos(properties.swayAngle);
  properties.mainP[2].z = properties.mainP[0].z - deltaZ2 * sin(properties.swayAngle);

  double deltaZ3 = properties.mainP[0].z - properties.mainP[3].z;
  properties.mainP[3].y = properties.mainP[0].y + side * deltaZ3 * cos(properties.swayAngle);
  properties.mainP[3].z = properties.mainP[0].z - deltaZ3 * sin(properties.swayAngle);

  properties.mainP[4].y = properties.mainP[0].y + side * properties.deltaZ * cos(properties.swayAngle);
  properties.mainP[4].z = properties.mainP[0].z - properties.deltaZ * sin(properties.swayAngle);
}

void Leg::fkine(double q0, double q1, double q2) {
	hipFkine(q0);
	mainFkine(q1, q2);
	properties.calculated_foot_pose = properties.mainP[4];
}

void Leg::hipIkine(coord foot) {
  int side;
  if (properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  properties.hipP[1].y = properties.hipP[0].y + side * properties.hipLengths[0] * cos(properties.hipSigma[0]);
  properties.hipP[1].z = properties.hipP[0].z - properties.hipLengths[0] * sin(properties.hipSigma[0]);

  properties.hipP[2].y = properties.hipP[1].y + side * properties.hipLengths[1] * cos(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])));
  properties.hipP[2].z = properties.hipP[1].z - properties.hipLengths[1] * sin(properties.hipSigma[0] - (M_PI - (properties.hipSigma[2] + properties.hipSigma[1])));

  properties.hipP[5].y = properties.hipP[1].y + side * properties.hipLengths[5] * cos(properties.hipSigma[0] - (M_PI - properties.hipSigma[1]));
  properties.hipP[5].z = properties.hipP[1].z - properties.hipLengths[5] * sin(properties.hipSigma[0] - (M_PI - properties.hipSigma[1]));

  double lr0 = distance(properties.hipP[2], properties.hipP[5]);

  if ((-side * foot.y) < fabs(properties.hipP[5].y)) { // Left of P4 on left leg, right side of P4 on right leg
    properties.swayAngle = fabs(atan((foot.z - properties.hipP[5].z) / (foot.y - properties.hipP[5].y)));
  } else {
    properties.swayAngle = M_PI - fabs(atan((foot.z - properties.hipP[5].z) / (foot.y - properties.hipP[5].y)));
  }

  properties.hipP[6].y = properties.hipP[5].y + side * properties.hipLengths[6] * cos(properties.swayAngle);
  properties.hipP[6].z = properties.hipP[5].z - properties.hipLengths[6] * sin(properties.swayAngle);

  properties.hipP[4].y = properties.hipP[5].y + side * properties.hipLengths[4] * cos(properties.swayAngle + properties.hipSigma[3]);
  properties.hipP[4].z = properties.hipP[5].z - properties.hipLengths[4] * sin(properties.swayAngle + properties.hipSigma[3]);

  properties.deltaZ = distance(foot, properties.hipP[6]);
  
  double lr1 = distance(properties.hipP[4], properties.hipP[2]);

  double alpha1 = angleCosineRule(properties.hipLengths[2], properties.hipLengths[3], lr1);
  double alpha2 = angleCosineRule(lr0, lr1, properties.hipLengths[4]);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(properties.hipLengths[3], properties.hipLengths[2], lr1);
  double beta2 = angleCosineRule(properties.hipLengths[4], lr1, lr0);
  double beta = beta1 + beta2;

  double gamma1 = angleCosineRule(lr1, properties.hipLengths[3], properties.hipLengths[2]); // Or M_PI - (alpah1 + beta1)
  double gamma2 = angleCosineRule(lr1, properties.hipLengths[4], lr0); // Or M_PI - (alpah2 + beta2)

  properties.hipP[3].y = properties.hipP[4].y + side * properties.hipLengths[3] * cos(properties.swayAngle + properties.hipSigma[3] + (M_PI - alpha));
  properties.hipP[3].z = properties.hipP[4].z - properties.hipLengths[3] * sin(properties.swayAngle + properties.hipSigma[3] + (M_PI - alpha));

  properties.calculated_q0 = rad2deg(-side * atan((properties.hipP[3].z - properties.hipP[2].z) / (properties.hipP[3].y - properties.hipP[2].y)));
}

void Leg::mainIkine(coord foot) {
  int side;
  if (properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  properties.mainP[0] = properties.hipP[6];


  properties.mainP[4] = foot;

  double lr1 = distance(properties.mainP[0], properties.mainP[4]);

  double gamma1 = angleCosineRule(lr1, properties.mainLengths[1], properties.mainLengths[3] + properties.mainLengths[4]);

  double lr2 = sideCosineRule(properties.mainLengths[1], properties.mainLengths[3], gamma1);

  double alpha1 = angleCosineRule(properties.mainLengths[3], properties.mainLengths[1], lr2);
  double alpha2 = angleCosineRule(properties.mainLengths[2], lr2, properties.mainLengths[0]);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(properties.mainLengths[1], properties.mainLengths[3], lr2);
  double beta2 = angleCosineRule(properties.mainLengths[0], lr2, properties.mainLengths[2]);
  double beta = beta1 + beta2;

  double gamma2 = angleCosineRule(lr2, properties.mainLengths[0], properties.mainLengths[2]); // or M_PI - (alpha2 + beta2)

  double theta;
  if (foot.x < properties.hipP[5].x) { // Foot behind P0
    theta = M_PI - fabs(atan((properties.mainP[4].z - properties.mainP[0].z) / (properties.mainP[4].x - properties.mainP[0].x)));
  } else {
    theta = fabs(atan((properties.mainP[4].z - properties.mainP[0].z) / (properties.mainP[4].x - properties.mainP[0].x)));
  }

  double iota = angleCosineRule(properties.mainLengths[3] + properties.mainLengths[4], properties.mainLengths[1], lr1);

  properties.calculated_q1 = theta + iota;
  properties.calculated_q2 = properties.calculated_q1 - alpha;
}

void Leg::ikine(coord foot) {
	hipIkine(foot);
	mainIkine(foot);
}

void Leg::zeroMotors() {
	hip_motor.zero();
	knee_motor_1.zero();
	knee_motor_2.zero();
}

void Leg::sendReceiveCycle() {
	
	if (status.inMotorMode) {
		knee_motor_1.in.kp_in = setKP;
		knee_motor_2.in.kp_in = setKP;
		knee_motor_1.in.kd_in = setKD;
		knee_motor_2.in.kd_in = setKD;
	} else {
		knee_motor_1.in.kp_in = 0;
		knee_motor_2.in.kp_in = 0;
		knee_motor_1.in.kd_in = 0;
		knee_motor_2.in.kd_in = 0;
	}
	convertOutputs();
	hip_motor.sendReceiveCycle();
	knee_motor_1.sendReceiveCycle();
	knee_motor_2.sendReceiveCycle();
}

void Leg::updateJointStatus() {
	//knee_motor_1.updateStatus();
	//knee_motor_2.updateStatus();
	//hip_motor.updateStatus();
	
	// Kineamtics here
	
	properties.current_q0 = hip_motor.out.converted_p_out;
	properties.current_q1 = knee_motor_1.out.converted_p_out;
	properties.current_q2 = knee_motor_2.out.converted_p_out;
	fkine(deg2rad(properties.current_q0), properties.current_q1, properties.current_q2);
	properties.current_foot_pose = properties.calculated_foot_pose;
	
	if (properties.trackGoalFootPose) {
		
	} else {
		properties.goal_q0 = properties.current_q0;
		properties.goal_q1 = properties.current_q1;
		properties.goal_q2 = properties.current_q2;
	}
}

void Leg::updatePoseStatus() {
	if (properties.trackGoalFootPose) {
		
	} else {
		properties.goal_foot_pose = properties.current_foot_pose;
	}
}

void Leg::convertJointAngles() {
	//knee_motor_1.convertOutputs();
	//knee_motor_2.convertOutputs();
	//hip_motor.convertOutputs();
	
	// This needs to be cleaned up to incorporate positiveDirection bool on motor classes
	if (properties.side == 1) { // right leg some values must be reversed
		knee_motor_1.out.converted_p_out = (knee_motor_1.out.p_out * 1) + knee_motor_1.properties.zeroPose;
		knee_motor_2.out.converted_p_out = ((knee_motor_2.out.p_out * 1) + knee_motor_2.properties.zeroPose) + (knee_motor_1.out.converted_p_out - knee_motor_1.properties.zeroPose);
		hip_motor.out.converted_p_out = ((hip_motor.out.p_out - hip_motor.properties.zeroPose) * -1); // shoudl change -1 and 1 to positiveDirection bit in motors
	} else {
		knee_motor_1.out.converted_p_out = (knee_motor_1.out.p_out * -1) + knee_motor_1.properties.zeroPose;
		knee_motor_2.out.converted_p_out = ((knee_motor_2.out.p_out * -1) + knee_motor_2.properties.zeroPose) + (knee_motor_1.out.converted_p_out - knee_motor_1.properties.zeroPose);
		hip_motor.out.converted_p_out = ((hip_motor.out.p_out - hip_motor.properties.zeroPose) * 1);
	}
}

void Leg::toggleGoalTracking(bool on) {
	properties.trackGoalFootPose = on;
}

void Leg::toggleGoalTracking() {
	if (properties.trackGoalFootPose) {
		properties.trackGoalFootPose = false;
	} else {
		properties.trackGoalFootPose = true;
	}
}

void Leg::motorsOn() {
	hip_motor.enterTorqueMode();
	knee_motor_1.enterMotorMode();
	knee_motor_2.enterMotorMode();
	status.inMotorMode = true;
}

void Leg::motorsOff() {
	hip_motor.exitTorqueMode();
	knee_motor_1.exitMotorMode();
	knee_motor_2.exitMotorMode();
	properties.trackGoalFootPose = false;
	status.inMotorMode = false;
}

void Leg::convertOutputs() {
	
	int tempSide;
	if (properties.side) {
		tempSide = 1;
	} else {
		tempSide = -1;
	}
	hip_motor.in.p_in = (-tempSide * properties.goal_q0 + hip_motor.properties.zeroPose);
	knee_motor_1.in.p_in = -tempSide * (knee_motor_1.properties.zeroPose - properties.goal_q1);

	knee_motor_2.in.p_in = -tempSide * ((knee_motor_2.properties.zeroPose - properties.goal_q2) - (knee_motor_1.properties.zeroPose - properties.goal_q1));
}

void Leg::moveToJointAngles() {
	
}

void Leg::moveToPose() {
	
}

// ALEX
Alex::Alex() {
	leg[0].knee_motor_1.can1 = can1;
	leg[0].knee_motor_2.can1 = can1;
	leg[1].knee_motor_1.can1 = can1;
	leg[1].knee_motor_2.can1 = can1;
}

void Alex::init() {
	DEBUG_SERIAL.begin(9600);
	dxl.begin(57600);
	dxl.setPortProtocolVersion(properties.dynamixel_protocol_version);
	pinMode(DXL_DIR_PIN, OUTPUT);
	digitalWrite(DXL_DIR_PIN, HIGH);
	can1.begin();
	can1.setBaudRate(1000000);
	
	leg[0].dxl = dxl;
	leg[1].dxl = dxl;
	leg[0].properties.side = 0;
	leg[1].properties.side = 1;
	leg[0].init();
	leg[1].init();
	
	
	// Turn off all motors
	//Set dynamixel operating mode
	//Get intial position of all motors and convert to angle from last known zero point
}

void Alex::zeroMotors() {
	leg[0].zeroMotors();
	leg[1].zeroMotors();
}

void Alex::unpack_TMotor_reply() {
  /// CAN REPLY PACKET STRUCTURE
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and 30 rad/s
  /// 12 bit current, between -40 and 40
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows: For each quantity, bit 0 is Lease Significant Bit
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  byte buf[8];
  byte len = 0;
  if (can1.read(in.statusMsg)) {
    unsigned long canId = in.statusMsg.id;

    for (int i = 0; i < 8; i++) {
      buf[i] = in.statusMsg.buf[i];
    }

    // Unpack ints from CAN buffer

    unsigned int p_int = (buf[1] << 8) | buf[2];
    unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
    unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert unsigned ints to floats
	
	unsigned int id = buf[0];
    if (id == leg[0].knee_motor_1.ID) {
      leg[0].knee_motor_1.out.p_out = leg[0].knee_motor_1.uint_to_float(p_int, leg[0].knee_motor_1.properties.P_MIN, leg[0].knee_motor_1.properties.P_MAX, 16);
      leg[0].knee_motor_1.out.v_out = leg[0].knee_motor_1.uint_to_float(v_int, leg[0].knee_motor_1.properties.V_MIN, leg[0].knee_motor_1.properties.V_MAX, 12);
      leg[0].knee_motor_1.out.t_out = leg[0].knee_motor_1.uint_to_float(i_int, -leg[0].knee_motor_1.properties.T_MAX, leg[0].knee_motor_1.properties.T_MAX, 12);
    } else if (id == leg[0].knee_motor_2.ID) {
      leg[0].knee_motor_2.out.p_out = leg[0].knee_motor_2.uint_to_float(p_int, leg[0].knee_motor_2.properties.P_MIN, leg[0].knee_motor_2.properties.P_MAX, 16);
	  leg[0].knee_motor_2.out.v_out = leg[0].knee_motor_2.uint_to_float(v_int, leg[0].knee_motor_2.properties.V_MIN, leg[0].knee_motor_2.properties.V_MAX, 12);
      leg[0].knee_motor_2.out.t_out = leg[0].knee_motor_2.uint_to_float(i_int, -leg[0].knee_motor_2.properties.T_MAX, leg[0].knee_motor_2.properties.T_MAX, 12);
    } else if (id == leg[1].knee_motor_1.ID) {
      leg[1].knee_motor_1.out.p_out = leg[1].knee_motor_1.uint_to_float(p_int, leg[1].knee_motor_1.properties.P_MIN, leg[1].knee_motor_1.properties.P_MAX, 16);
      leg[1].knee_motor_1.out.v_out = leg[1].knee_motor_1.uint_to_float(v_int, leg[1].knee_motor_1.properties.V_MIN, leg[1].knee_motor_1.properties.V_MAX, 12);
      leg[1].knee_motor_1.out.t_out = leg[1].knee_motor_1.uint_to_float(i_int, -leg[1].knee_motor_1.properties.T_MAX, leg[1].knee_motor_1.properties.T_MAX, 12);
    } else if (id == leg[1].knee_motor_2.ID) {
      leg[1].knee_motor_2.out.p_out = leg[1].knee_motor_2.uint_to_float(p_int, leg[1].knee_motor_2.properties.P_MIN, leg[1].knee_motor_2.properties.P_MAX, 16);
      leg[1].knee_motor_2.out.v_out = leg[1].knee_motor_2.uint_to_float(v_int, leg[1].knee_motor_2.properties.V_MIN, leg[1].knee_motor_2.properties.V_MAX, 12);
      leg[1].knee_motor_2.out.t_out = leg[1].knee_motor_2.uint_to_float(i_int, -leg[1].knee_motor_2.properties.T_MAX, leg[1].knee_motor_2.properties.T_MAX, 12);
    }
  }
}

void Alex::sendReceiveCycle() {
	leg[0].sendReceiveCycle();
	leg[1].sendReceiveCycle();
}

void Alex::updateJointStatus() {
	leg[0].updateJointStatus();
	leg[1].updateJointStatus();
}

void Alex::updatePoseStatus() {
	leg[0].updatePoseStatus();
	leg[1].updatePoseStatus();
}

void Alex::getInitalPose() {
	//leg[0].hipFkine(
}

void Alex::convertJointAngles() {
	leg[0].convertJointAngles();
	leg[1].convertJointAngles();
}

void Alex::toggleGoalTracking() {
	if (properties.trackGoalFootPose) {
		properties.trackGoalFootPose = false;
	} else {
		properties.trackGoalFootPose = true;
	}
	leg[0].toggleGoalTracking(properties.trackGoalFootPose);
	leg[1].toggleGoalTracking(properties.trackGoalFootPose);
}

void Alex::ES_check() {
	//if (leg[0].status.inMotorMode) {
		if (fabs(leg[0].knee_motor_1.out.v_out) > 3 || fabs(leg[0].knee_motor_2.out.v_out) > 3) {
			leg[0].motorsOff();
		}
	//}
	
	if (leg[1].status.inMotorMode) {
		if (fabs(leg[1].knee_motor_1.out.v_out) > 3 || fabs(leg[1].knee_motor_2.out.v_out) > 3) {
			leg[1].motorsOff();
		}
	}

}

void Alex::poseCheck() {
	if (leg[0].properties.goal_foot_pose.x >= maxPoseX) {
		leg[0].properties.goal_foot_pose.x = maxPoseX;
	} else if (leg[0].properties.goal_foot_pose.x <= minPoseX) {
		leg[0].properties.goal_foot_pose.x = minPoseX;
	}
	
	if (leg[0].properties.goal_foot_pose.y >= maxPoseY) {
		leg[0].properties.goal_foot_pose.y = maxPoseY;
	} else if (leg[0].properties.goal_foot_pose.y <= minPoseY) {
		leg[0].properties.goal_foot_pose.y = minPoseY;
	}
	
	if (leg[0].properties.goal_foot_pose.z >= maxPoseZ) {
		leg[0].properties.goal_foot_pose.z = maxPoseZ;
	} else if (leg[0].properties.goal_foot_pose.z <= minPoseZ) {
		leg[0].properties.goal_foot_pose.z = minPoseZ;
	}
	
	if (leg[1].properties.goal_foot_pose.x >= maxPoseX) {
		leg[1].properties.goal_foot_pose.x = maxPoseX;
	} else if (leg[1].properties.goal_foot_pose.x <= minPoseX) {
		leg[1].properties.goal_foot_pose.x = minPoseX;
	}
	
	if (leg[1].properties.goal_foot_pose.y <= -maxPoseY) {
		leg[1].properties.goal_foot_pose.y = -maxPoseY;
	} else if (leg[1].properties.goal_foot_pose.y >= -minPoseY) {
		leg[1].properties.goal_foot_pose.y = -minPoseY;
	}
	
	if (leg[1].properties.goal_foot_pose.z >= maxPoseZ) {
		leg[1].properties.goal_foot_pose.z = maxPoseZ;
	} else if (leg[1].properties.goal_foot_pose.z <= minPoseZ) {
		leg[1].properties.goal_foot_pose.z = minPoseZ;
	}

}

void Alex::motorsOn() {
	status.inMotorMode = true;
	leg[0].motorsOn();
	leg[1].motorsOn();
	
}

void Alex::motorsOff() {
	status.inMotorMode = false;
	leg[0].motorsOff();
	leg[1].motorsOff();
}

void Alex::calcJointAngles() {
	if (leg[0].properties.trackGoalFootPose) {
		leg[0].ikine(leg[0].properties.goal_foot_pose);
		leg[0].properties.goal_q0 = leg[0].properties.calculated_q0;
		leg[0].properties.goal_q1 = leg[0].properties.calculated_q1;
		leg[0].properties.goal_q2 = leg[0].properties.calculated_q2;
	}
	
	if (leg[1].properties.trackGoalFootPose) {
		leg[1].ikine(leg[1].properties.goal_foot_pose);
		leg[1].properties.goal_q0 = leg[1].properties.calculated_q0;
		leg[1].properties.goal_q1 = leg[1].properties.calculated_q1;
		leg[1].properties.goal_q2 = leg[1].properties.calculated_q2;
	}
}

//ALEX CYCLE
// 0: Receive Status from motors
// 1: Update internal parameters
// 2: Receive commands from external
// 3: Check commands and manipulate internals
// 4: Move
Alex_cycle::Alex_cycle() {
	stepNo = 0;
}

void Alex_cycle::step_0() {
	
}

void Alex_cycle::step_1() {
	
}

void Alex_cycle::step_2() {
	
}

void Alex_cycle::step_3() {
	
}

void Alex_cycle::step_4() {
	
}
