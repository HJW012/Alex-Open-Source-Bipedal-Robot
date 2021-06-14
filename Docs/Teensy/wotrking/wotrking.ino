#include <FlexCAN_T4.h>


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

// Motor Variables - Send to Motor
float globalP = 0.0f; //Position
float globalV = 10.0f; //Speed
float globalKP = 500.0f; //Stiffness
float globalKD = 0.0f; //Damper
float globalT = 18.0f; //Torque


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
CAN_message_t canMsg2;
float p_out;
float v_out;
float t_out;

void setup(void) {
  Serial.begin(9600);
  can1.begin();
  can1.setBaudRate(1000000);
  msg.id = 0x001;
  msg.len = 8;
  delay(500);
  msg.buf[0] = 0xFF;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0xFF;
  msg.buf[3] = 0xFF;
  msg.buf[4] = 0xFF;
  msg.buf[5] = 0xFF;
  msg.buf[6] = 0xFF;
  msg.buf[7] = 0xFD;
  can1.write(msg);


  pack_cmd();
}

void loop() {

  unpack_reply();
  //globalP = p_out;
  //globalV = v_out;
  //globalT = t_out;
  //pack_cmd();
}

void pack_cmd() {
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
  float p_des = constrain(globalP, P_MIN, P_MAX);
  float v_des = constrain(globalV, V_MIN, V_MAX);
  float kp = constrain(globalKP, KP_MIN, KP_MAX);
  float kd = constrain(globalKD, KD_MIN, KD_MAX);
  float t_ff = constrain(globalT, T_MIN, T_MAX);

  // Convert floats to unsigned ints
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // Pack ints into CAN buffer
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;


  msg.id  = 0x001;
  msg.len = 8;
  for (int i = 0; i < 8; i++) {
    msg.buf[i] = buf[i];
  }
  can1.write(msg);
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
  byte buf[8];
  byte len = 0;
  if (can1.read(canMsg2)) {

    unsigned long canId = canMsg2.id;
    Serial.print("Raw HEX Response: ");
    for (int i = 0; i < 8; i++) {
      buf[i] = canMsg2.buf[i];
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Unpack ints from CAN buffer
    unsigned int id = buf[0];
    unsigned int p_int = (buf[1] << 8) | buf[2];
    unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
    unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert unsigned ints to floats
    p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
    t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    Serial.print("Pose: ");
    Serial.print(p_out);
    Serial.print("\tVel: ");
    Serial.print(v_out);
    Serial.print("\tTorque: ");
    Serial.println(t_out);
  }
}




unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  /*if (bits == 12) {
    pgg = (unsigned int) ((x - offset) * 4095.0 / span);
    }
    if (bits == 16) {
    pgg = (unsigned int) ((x - offset) * 65535.0 / span);
    }*/
  pgg = (unsigned int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
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
