#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t commandMsg;
CAN_message_t statusMsg;

//Limb Lengths
#define l1 100.0f
#define l2 310.0f
#define l3 315.0f
#define l4 105.0f
#define l5 150.0f

#define loopDelayMicros 500
#define statusDelay 500

// Leg1
#define motor_11_ID 11 // Parent ID
#define motor_12_ID 12 // Child ID

// Leg2
#define motor_21_ID 21 // Parent ID
#define motor_22_ID 22 // Child ID

// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Set Values - send to motor
float p_in[] = {0.0f, 0.0f, 0.0f, 0.0f}; //Position
float v_in[] = {0.0f, 0.0f, 0.0f, 0.0f}; //Speed
float kp_in[] = {100.0f, 100.0f, 100.0f, 100.0f}; //Stiffness
float kd_in[] = {1.0f, 1.0f, 1.0f, 1.0f}; //Damper
float t_in[] = {0.0f, 0.0f, 0.0f, 0.0f}; //Torque

// Measured Values - received from motor
float p_out[] = {0.0f, 0.0f, 0.0f, 0.0f};
float v_out[] = {0.0f, 0.0f, 0.0f, 0.0f};
float t_out[] = {0.0f, 0.0f, 0.0f, 0.0f};

bool inMotorMode[] = {false, false, false, false};
bool commandSent = false; // Hopefully only send one command msg per loop - this decides whetehr to send status msg or not

// Pin Definitions
#define motorModeButton 17
#define zeroMotorsButton 16
#define moveFootUpButton 15
#define moveFootDownButton 14
#define moveIncrement 5.0f
bool motorsZeroed = false;

// Debounce bools
bool debounce1 = false;
bool debounce2 = false;
bool debounce3 = false;
bool debounce4 = false;

// Kinematics Vairables
float minZ = 200;
float maxZ = 450;
float goalZ = 327.02;
float goalX = -1.467;
#define radToDeg 180/PI
#define startingQ2 3 * PI / 4
#define startingQ1 PI / 4

void setup() {
  Serial.begin(9600);

  can1.begin();
  can1.setBaudRate(1000000);
  commandMsg.len = 8;
  statusMsg.len = 8;

  pinMode(motorModeButton, INPUT_PULLUP);
  pinMode(zeroMotorsButton, INPUT_PULLUP);
  pinMode(moveFootUpButton, INPUT_PULLUP);
  pinMode(moveFootDownButton, INPUT_PULLUP);

  t_in[0] = 0;
  t_in[1] = 0;
  v_in[0] = 0;
  v_in[1] = 0;
  kd_in[0] = 1;
  kd_in[1] = 1;
  kp_in[0] = 100;
  kp_in[1] = 100;
}

void loop() {
  Serial.println("P0: " + String(p_out[0]));
  Serial.println("P1: " + String(p_out[1]));
  Serial.println("P2: " + String(p_out[2]));
  Serial.println("P3: " + String(p_out[3]));
  handleButtons();

  kneeIkine("left");
  kneeIkine("right");


  pack_cmd(motor_11_ID);
  delayMicroseconds(statusDelay);
  unpack_reply();

  pack_cmd(motor_12_ID);
  delayMicroseconds(statusDelay);
  unpack_reply();

  pack_cmd(motor_21_ID);
  delayMicroseconds(statusDelay);
  unpack_reply();

  pack_cmd(motor_22_ID);
  delayMicroseconds(statusDelay);
  unpack_reply();


  delay(10);
}

void handleButtons() {
  if (!digitalRead(motorModeButton) && !debounce1) {
    if (motorsZeroed) {
      toggleMotorMode(motor_11_ID);
      toggleMotorMode(motor_12_ID);
      toggleMotorMode(motor_21_ID);
      toggleMotorMode(motor_22_ID);
    } else {
      Serial.println("First zero motors");
    }

    debounce1 = true;
  } else if (digitalRead(motorModeButton)) {
    debounce1 = false;
  }

  if (!digitalRead(zeroMotorsButton) && !debounce2) {
    if (!inMotorMode[0] && !inMotorMode[1]) {
      zeroMotor(motor_11_ID);
      zeroMotor(motor_12_ID);
      zeroMotor(motor_21_ID);
      zeroMotor(motor_22_ID);
      motorsZeroed = true;
    }

    debounce2 = true;
  } else if (digitalRead(zeroMotorsButton)) {
    debounce2 = false;
  }

  if (!digitalRead(moveFootUpButton)) {
    if (goalZ - moveIncrement >= minZ) {
      goalZ -= moveIncrement;
    }

    debounce3 = true;
  } else if (digitalRead(moveFootUpButton)) {
    debounce3 = false;
  }

  if (!digitalRead(moveFootDownButton)) {
    if (goalZ + moveIncrement <= maxZ) {
      goalZ += moveIncrement;
    }

    debounce3 = true;
  } else if (digitalRead(moveFootDownButton)) {
    debounce3 = false;
  }
}

void toggleMotorMode(uint32_t ID) {
  commandMsg.id = ID;
  int motorIndex = 0;

  if (ID == motor_11_ID) {
    motorIndex = 0;
  } else if (ID == motor_12_ID) {
    motorIndex = 1;
  } else if (ID == motor_21_ID) {
    motorIndex = 2;
  } else if (ID == motor_22_ID) {
    motorIndex = 3;
  }

  if (!inMotorMode[motorIndex]) {
    commandMsg.buf[0] = 0xFF;
    commandMsg.buf[1] = 0xFF;
    commandMsg.buf[2] = 0xFF;
    commandMsg.buf[3] = 0xFF;
    commandMsg.buf[4] = 0xFF;
    commandMsg.buf[5] = 0xFF;
    commandMsg.buf[6] = 0xFF;
    commandMsg.buf[7] = 0xFC;
  } else {
    commandMsg.buf[0] = 0xFF;
    commandMsg.buf[1] = 0xFF;
    commandMsg.buf[2] = 0xFF;
    commandMsg.buf[3] = 0xFF;
    commandMsg.buf[4] = 0xFF;
    commandMsg.buf[5] = 0xFF;
    commandMsg.buf[6] = 0xFF;
    commandMsg.buf[7] = 0xFD;
  }
  inMotorMode[motorIndex] = !inMotorMode[motorIndex];
  can1.write(commandMsg);
  //  delayMicroseconds(loopDelayMicros);
  delay(5);
}

void zeroMotor(uint32_t ID) {
  commandMsg.id = ID;

  commandMsg.buf[0] = 0xFF;
  commandMsg.buf[1] = 0xFF;
  commandMsg.buf[2] = 0xFF;
  commandMsg.buf[3] = 0xFF;
  commandMsg.buf[4] = 0xFF;
  commandMsg.buf[5] = 0xFF;
  commandMsg.buf[6] = 0xFF;
  commandMsg.buf[7] = 0xFE;

  can1.write(commandMsg);
  delay(5);
}

void pack_cmd(uint32_t ID) {
  int motorIndex = 2;
  if (ID == motor_11_ID) {
    motorIndex = 0;
  } else if (ID == motor_12_ID) {
    motorIndex = 1;
  } else if (ID == motor_21_ID) {
    motorIndex = 2;
  } else if (ID == motor_22_ID) {
    motorIndex = 3;
  }

  byte buf[8];

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
  float p_des = constrain(p_in[motorIndex], P_MIN, P_MAX);
  float v_des = constrain(v_in[motorIndex], V_MIN, V_MAX);
  float kp = constrain(kp_in[motorIndex], KP_MIN, KP_MAX);
  float kd = constrain(kd_in[motorIndex], KD_MIN, KD_MAX);
  float t_ff = constrain(t_in[motorIndex], T_MIN, T_MAX);

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // Pack ints into CAN buffer
  commandMsg.id = ID;
  commandMsg.buf[0] = p_int >> 8;
  commandMsg.buf[1] = p_int & 0xFF;
  commandMsg.buf[2] = v_int >> 4;
  commandMsg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  commandMsg.buf[4] = kp_int & 0xFF;
  commandMsg.buf[5] = kd_int >> 4;
  commandMsg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  commandMsg.buf[7] = t_int & 0xFF;
  can1.write(commandMsg);

}

void unpack_reply() {
  int motorIndex = 0;

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
  if (can1.read(statusMsg)) {
    unsigned long canId = statusMsg.id;

    for (int i = 0; i < 8; i++) {
      buf[i] = statusMsg.buf[i];
    }

    // Unpack ints from CAN buffer
    unsigned int id = buf[0];
    if (id == motor_11_ID) {
      motorIndex = 0;
    } else if (id == motor_12_ID) {
      motorIndex = 1;
    } else if (id == motor_21_ID) {
      motorIndex = 2;
    } else if (id == motor_22_ID) {
      motorIndex = 3;
    }

    unsigned int p_int = (buf[1] << 8) | buf[2];
    unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
    unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert unsigned ints to floats
    p_out[motorIndex] = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v_out[motorIndex] = uint_to_float(v_int, V_MIN, V_MAX, 12);
    t_out[motorIndex] = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  }
}

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
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

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
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

double distance(double x1, double y1, double x2, double y2) {
  double dist = abs(sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2)));
  return dist;
}

double angleCosineRule(double a, double b, double c) {
  double A = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
  return A;
}

double sideCosineRule(double b, double c, double A) {
  double a = sqrt(pow(b, 2) + pow(c, 2) - 2 * b * c * cos(A));
  return a;
}

bool kneeIkine(String prefix) {
  double lr1 = distance(0, 0, goalX, goalZ);
  double alpha = angleCosineRule(lr1, l2, l4 + l5);
  double lr2 = sideCosineRule(l2, l4, alpha);
  double beta = angleCosineRule(lr2, l1, l3);
  double delta1 = angleCosineRule(l2, l4, lr2);
  double delta2 = angleCosineRule(l1, lr2, l3);
  double gamma1 = angleCosineRule(l4, l2, lr2);
  double gamma2 = angleCosineRule(l3, lr2, l1);
  double gamma = gamma1 + gamma2;
  double iota = angleCosineRule(l4 + l5, l2, lr1);
  double sigma;

  if (goalX > 0)  {
    sigma = -atan((-goalZ - 0) / (goalX - 0));
  } else {
    sigma = PI - atan((-goalZ - 0) / (goalX - 0));
  }

  double q2 = (sigma + iota); // Angle to long parent link from horizontal forward - positive rotating backwards - p_in 0
  //  double q2_0ffset =
  double q1 = q2 - gamma;  //  Angle to short child link from horizontal foward - positive rotating backwards - p_in 1

  //    Serial.print("Q1: ");
  //    Serial.println(q1 * radToDeg);
  //    Serial.print("Q2: ");
  //    Serial.println(q2 * radToDeg);

  convertToTmotorPose(prefix, q1, q2);
}

bool convertToTmotorPose(String prefix, double q1, double q2) {
  // Assuming motors have been zeroed at the defined starting poses
  // Motors rotate clockwise positive
  double localQ1;
  double localQ2;

  localQ1 = q1;
  localQ2 = q2;
  double p_in_0 = startingQ2 - localQ2;
  double p_in_1 = startingQ1 - localQ1 - p_in_0;


  if (prefix == "left") {

    p_in[0] = p_in_0;
    p_in[1] = p_in_1;
  } else if (prefix == "right") {
    p_in[2] = -p_in_0;
    p_in[3] = -p_in_1;
  }





}
