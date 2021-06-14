#include <FlexCAN_T4.h>

#define blank_ID 0x000
#define motor_ID_1 0x001
#define motor_ID_2 0x002

//#define torque_threshold 0.02f
float torque_threshold = 0.02;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t motor_command;
CAN_message_t motor_reply;

// Telepresence control gains
float KP = 100000;
float KD = 100000;
float K = 1000;


// Motor Limits
#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Motor 1 Command Values
float p1_in = 0.0f;
float v1_in = 0.0f;
float kp1_in = 0.0f;
float kd1_in = 0.0f;
float t1_in = 0.0f;

// Motor 1 Reply Values
float p1_out = 0.0f;
float v1_out = 0.0f;
float t1_out = 0.0f;

// Motor 2 Command Values
float p2_in = 0.0f;
float v2_in = 0.0f;
float kp2_in = 0.0f;
float kd2_in = 0.0f;
float t2_in = 0.0f;

// Motor 2 Reply Values
float p2_out = 0.0f;
float v2_out = 0.0f;
float t2_out = 0.0f;

#define motor_step 0.01f

void setup() {
  Serial.begin(9600);
  can1.begin();
  can1.setBaudRate(1000000);

  zeroMotor(motor_ID_1);
  zeroMotor(motor_ID_2);
  delay(10);
  enterMotorMode(motor_ID_1);
  enterMotorMode(motor_ID_2);
}

void loop() {
  if (can1.read(motor_reply)) {
    unpack_reply();
  }
  control();
  //displayData();
  delay(1);
}

void zeroMotors() {
  zeroMotor(motor_ID_1);
  zeroMotor(motor_ID_2);
}

void zeroMotor(unsigned int motor_ID) {
  motor_command.id = motor_ID;
  motor_command.buf[0] = 0xFF;
  motor_command.buf[1] = 0xFF;
  motor_command.buf[2] = 0xFF;
  motor_command.buf[3] = 0xFF;
  motor_command.buf[4] = 0xFF;
  motor_command.buf[5] = 0xFF;
  motor_command.buf[6] = 0xFF;
  motor_command.buf[7] = 0xFE;

  can1.write(motor_command);
}

void resetCommand() {
  motor_command.buf[0] = 0x00;
  motor_command.buf[1] = 0x00;
  motor_command.buf[2] = 0x00;
  motor_command.buf[3] = 0x00;
  motor_command.buf[4] = 0x00;
  motor_command.buf[5] = 0x00;
  motor_command.buf[6] = 0x00;
  motor_command.buf[7] = 0x00;
}

void resetParams(unsigned int motorID) {
  switch (motorID) {
    case motor_ID_1:
      p1_in = 0;
      v1_in = 0;
      kp1_in = 0;
      kd1_in = 0;
      t1_in = 0;
      break;

    case motor_ID_2:
      p2_in = 0;
      v2_in = 0;
      kp2_in = 0;
      kd2_in = 0;
      t2_in = 0;
      break;
  }
}

void enterMotorMode(unsigned int motorID) {
  motor_command.id = motorID;
  motor_command.len = 8;
  motor_command.buf[0] = 0xFF;
  motor_command.buf[1] = 0xFF;
  motor_command.buf[2] = 0xFF;
  motor_command.buf[3] = 0xFF;
  motor_command.buf[4] = 0xFF;
  motor_command.buf[5] = 0xFF;
  motor_command.buf[6] = 0xFF;
  motor_command.buf[7] = 0xFC;


  can1.write(motor_command);
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
  //pgg = (unsigned int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
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

void torque_control() {
  resetParams(motor_ID_1);
  Serial.println(t1_out);
  if (t1_out >= torque_threshold || t1_out <= -torque_threshold) {
    t1_in = t1_out;
    Serial.println("NOW");
  } else {
    t1_in = 0;
  }

  sendCommand(motor_ID_1);
}

void position_control() { // PD controller to adust step increment based on torque feedback
  if (abs(t1_in) >= torque_threshold) {

  }
}

void control() {
  t1_in = KP*(p2_out - p1_out) + KD*(v2_out - v1_out) - K*(v1_out);
  t2_in = KP*(p1_out - p2_out) + KD*(v1_out - v2_out) - K*(v2_out);
  sendCommand(motor_ID_1);
  sendCommand(motor_ID_2); 
}

void sendCommand(unsigned int motorID) {
  switch (motorID) {
    case motor_ID_1:
      pack_cmd(motor_ID_1);
      can1.write(motor_command);
      break;

    case motor_ID_2:
      pack_cmd(motor_ID_2);
      can1.write(motor_command);
      break;

    case blank_ID:
      resetCommand();
      motor_command.id = motor_ID_1;
      can1.write(motor_command);
      motor_command.id = motor_ID_2;
      can1.write(motor_command);
      break;

    default:
      break;
  }
}

void pack_cmd(unsigned int motorID) {
  float p_des;
  float v_des;
  float kp;
  float kd;
  float t_ff;

  // Limit data to be within bounds
  switch (motorID) {
    case motor_ID_1:
      p_des = constrain(p1_in, P_MIN, P_MAX);
      v_des = constrain(v1_in, V_MIN, V_MAX);
      kp = constrain(kp1_in, KP_MIN, KP_MAX);
      kd = constrain(kd1_in, KD_MIN, KD_MAX);
      t_ff = constrain(t1_in, T_MIN, T_MAX);
      break;

    case motor_ID_2:
      p_des = constrain(p2_in, P_MIN, P_MAX);
      v_des = constrain(v2_in, V_MIN, V_MAX);
      kp = constrain(kp2_in, KP_MIN, KP_MAX);
      kd = constrain(kd2_in, KD_MIN, KD_MAX);
      t_ff = constrain(t2_in, T_MIN, T_MAX);
      break;
  }

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // Pack ints into CAN buffer
  motor_command.id = motorID;
  motor_command.len = 8;
  motor_command.buf[0] = p_int >> 8;
  motor_command.buf[1] = p_int & 0xFF;
  motor_command.buf[2] = v_int >> 4;
  motor_command.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  motor_command.buf[4] = kp_int & 0xFF;
  motor_command.buf[5] = kd_int >> 4;
  motor_command.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  motor_command.buf[7] = t_int & 0xFF;
}

void unpack_reply() {
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
  byte len = 0;
  unsigned long canID = motor_reply.id;
  /*Serial.print("Raw HEX Response: ");
    for (int i = 0; i < 8; i++) {
    Serial.print(tmotor_reply[i], HEX);
    Serial.print(" ");
    }
    Serial.println();*/

  // Unpack ints from CAN buffer
  unsigned int id = motor_reply.buf[0];
  unsigned int p_int = (motor_reply.buf[1] << 8) | motor_reply.buf[2];
  unsigned int v_int = (motor_reply.buf[3] << 4) | (motor_reply.buf[4] >> 4);
  unsigned int i_int = ((motor_reply.buf[4] & 0xF) << 8) | motor_reply.buf[5];

  // Convert unsigned ints to floats
  switch (id) {
    case motor_ID_1:
      p1_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
      v1_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
      t1_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
      break;

    case motor_ID_2:
      p2_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
      v2_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
      t2_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
      break;

    default:
      break;
  }
}

void displayData() {
  Serial.print(p1_in);
  Serial.print(" ");
  Serial.print(p1_out);
  Serial.print(" ");
  Serial.print(v1_in);
  Serial.print(" ");
  Serial.print(v1_out);
  Serial.print(" ");
  Serial.print(t1_in);
  Serial.print(" ");
  Serial.print(t1_out);
  Serial.println();

}
