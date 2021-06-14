#include <AlexLib.h>



#define DEBUG_SERIAL Serial
#define DEBUG(x) Serial.println(x)
//const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t commandMsg;
CAN_message_t statusMsg;

// Dynamixel Definitions
#define leg_1_hip_motor_ID 13
#define leg_2_hip_motor_ID 23


#define OPERATING_MODE_ADDR         11
#define OPERATING_MODE_ADDR_LEN     1
#define TORQUE_ENABLE_ADDR          64
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    65
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          116
#define GOAL_POSITION_ADDR_LEN      4
#define PRESENT_POSITION_ADDR       132
#define PRESENT_POSITION_ADDR_LEN   4
#define POSITION_CONTROL_MODE       3
#define TIMEOUT 10    //default communication timeout 10ms


// TMotor Definitions
#define leg_1_knee_motor_1_ID 11
#define leg_1_knee_motor_2_ID 12
#define leg_2_knee_motor_1_ID 21
#define leg_2_knee_motor_2_ID 22

// Pin Definitions
#define zeroMotorButton 8
#define motorModeButton 9

#define xIncrement 5
#define yIncrement 5
#define zIncrement 2


Alex alex;

class joystick {
  private:
    bool debounce = false;
  public:
    int xPin;
    int yPin;
    int zPin;
    int buttonPin;

    int xVal = 0;
    int yVal = 0;
    int zVal = 0;
    bool buttonState = false;

    int centreX = 0;
    int centreY = 0;
    int centreZ = 0;

    int increment = 100;

    joystick(int pinX, int pinY, int pinZ, int pinButton) {
      xPin = pinX;
      yPin = pinY;
      zPin = pinZ;
      buttonPin = pinButton;
    }

    void init() {
      pinMode(xPin, INPUT);
      pinMode(yPin, INPUT);
      pinMode(zPin, INPUT);
      pinMode(buttonPin, INPUT_PULLUP);
    }

    void setCentre(int x, int y, int z) {
      centreX = x;
      centreY = y;
      centreZ = z;
    }

    int getX() {
      xVal = analogRead(xPin);
      return xVal;
    }

    int getY() {
      yVal = analogRead(yPin);
      return yVal;
    }

    int getZ() {
      zVal = analogRead(zPin);
      return zVal;
    }

    bool getButton() {
      if (!digitalRead(buttonPin) && !debounce) {
        buttonState = true;
      } else if (digitalRead(buttonPin)) {
        buttonState = false;
      }

      return buttonState;
    }

    void getData() {
      getX();
      getY();
      getZ();
      getButton();
    }
};

joystick leftJoystick(19, 18, 20, 21);
joystick rightJoystick(15, 14, 16, 17);
bool debounce1 = false;
bool debounce2 = false;
using namespace ControlTableItem;

void setup() {
  DEBUG_SERIAL.begin(9600);
  alex.dxl.begin(57600); // Default baud rate for dynamixels
  alex.init();


  alex.properties.p[0] = coord(0, 0, 0); // Centre
  alex.properties.p[1] = coord(0, 32.497, -26.038); // Left
  alex.properties.p[2] = coord(0, -32.497, -26.038); // Right

  alex.leg[0].properties.hipP[0] = alex.properties.p[1];
  alex.leg[1].properties.hipP[0] = alex.properties.p[2];
  alex.leg[0].properties.side = 0;
  alex.leg[1].properties.side = 1;

  while (!Serial);
  alex.leg[0].hipFkine(deg2rad(15));
  alex.leg[0].mainFkine(deg2rad(23.698), deg2rad(142.333));
  DEBUG("FKine Foot coords");
  DEBUG(String(alex.leg[0].properties.mainP[4].x) + ", " + String(alex.leg[0].properties.mainP[4].y) + ", " + String(alex.leg[0].properties.mainP[4].z));
  /*int j = 0;
    String tempString;
    for (auto x : alex.leg[0].properties.hipP) {
    tempString = String(j) + ": " + String(x.x) + ", " + String(x.y) + ", " + String(x.z);
    DEBUG(tempString);
    j++;
    }
    DEBUG("Main: ");
    j = 0;
    for (auto x : alex.leg[0].properties.mainP) {
    tempString = String(j) + ": " + String(x.x) + ", " + String(x.y) + ", " + String(x.z);
    DEBUG(tempString);
    j++;
    }*/

  coord footPose;
  footPose.x = -1.467;
  footPose.y = 175.348;
  footPose.z = -502.241;

  //footPose = alex.leg[0].properties.mainP[4];

  hipIkine(alex.leg[0], footPose);
  mainIkine(alex.leg[0], footPose);
  DEBUG("Ikine Joint Angles");
  DEBUG(String(rad2deg(alex.leg[0].properties.q0)));
  DEBUG(String(rad2deg(alex.leg[0].properties.q1)));
  DEBUG(String(rad2deg(alex.leg[0].properties.q2)));

  leftJoystick.init();
  leftJoystick.setCentre(624, 563, 571);
  rightJoystick.init();
  rightJoystick.setCentre(556, 540, 543);


  pinMode(zeroMotorButton, INPUT);
  pinMode(motorModeButton, INPUT);
  //dxl.setOperatingMode(leg_1_hip_motor_ID, OP_POSITION);
  //dxl.writeControlTableItem(PROFILE_VELOCITY, leg_1_hip_motor_ID, 30);
}

elapsedMillis averageTime;
double average[] = {0, 0, 0, 0, 0, 0};
int idx = 0;
void loop() {
  leftJoystick.getData();
  rightJoystick.getData();
  handleButtons();
  handleJoysticks();


  DEBUG("LEFT: " + String(leftJoystick.xVal) + ", " + String(leftJoystick.yVal) + ", " + String(leftJoystick.zVal) + ", " + String(leftJoystick.getButton()));
  DEBUG("RIGHT: " + String(rightJoystick.xVal) + ", " + String(rightJoystick.yVal) + ", " + String(rightJoystick.zVal) + ", " + String(rightJoystick.getButton()));
  DEBUG();

  delay(15);
}

void initDynamixels() {
  //dxl.
}

void handleJoysticks() {
  leftJoystick.getData();
  rightJoystick.getData();

  // LEFT
  if (abs(leftJoystick.xVal - leftJoystick.centreX) >= leftJoystick.increment) {
    int multiplyer = (leftJoystick.xVal - leftJoystick.centreX) / leftJoystick.increment;
    alex.leg[0].in.goalFootPose.x += (xIncrement * multiplyer);
  }

  if (abs(leftJoystick.yVal - leftJoystick.centreY) >= leftJoystick.increment) {
    int multiplyer = (leftJoystick.yVal - leftJoystick.centreY) / leftJoystick.increment;
    alex.leg[0].in.goalFootPose.y += (yIncrement * multiplyer);
  }

  if (abs(leftJoystick.zVal - leftJoystick.centreZ) >= leftJoystick.increment) {
    int multiplyer = (leftJoystick.zVal - leftJoystick.centreZ) / leftJoystick.increment;
    alex.leg[0].in.goalFootPose.z += (zIncrement * multiplyer);
  }

  // RIGHT
  if (abs(rightJoystick.xVal - rightJoystick.centreX) >= rightJoystick.increment) {
    int multiplyer = (rightJoystick.xVal - rightJoystick.centreX) / rightJoystick.increment;
    alex.leg[1].in.goalFootPose.x += (xIncrement * multiplyer);
  }

  if (abs(rightJoystick.yVal - rightJoystick.centreY) >= rightJoystick.increment) {
    int multiplyer = (rightJoystick.yVal - rightJoystick.centreY) / rightJoystick.increment;
    alex.leg[1].in.goalFootPose.y += (yIncrement * multiplyer);
  }

  if (abs(rightJoystick.zVal - rightJoystick.centreZ) >= rightJoystick.increment) {
    int multiplyer = (rightJoystick.zVal - rightJoystick.centreZ) / rightJoystick.increment;
    alex.leg[1].in.goalFootPose.z += (zIncrement * multiplyer);
  }
}

void handleButtons() {
  if (!digitalRead(zeroMotorButton) && !debounce1) {
    debounce1 = true;
    zeroMotors();
  } else if (digitalRead(zeroMotorButton)) {
    debounce1 = false;
  }

  if (!digitalRead(motorModeButton) && !debounce2) {
    debounce2 = true;
  } else if (digitalRead(motorModeButton)) {
    debounce2 = false;
  }
}

void zeroMotors() {
  // Leg
  DEBUG("Move motor with ID 11 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[0].knee_motor_1.zero();
  delay(5);
  DEBUG("Move motor with ID 12 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[0].knee_motor_2.zero();
  delay(5);
  DEBUG("Move motor with ID 13 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[0].hip_motor.zero();
  delay(5);

  // Right
  DEBUG("Move motor with ID 21 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[1].knee_motor_1.zero();
  delay(5);
  DEBUG("Move motor with ID 22 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[1].knee_motor_2.zero();
  delay(5);
  DEBUG("Move motor with ID 23 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[1].hip_motor.zero();
  delay(5);
}

void hipIkine(Leg& leg, coord foot) {
  int side;
  if (leg.properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  leg.properties.hipP[1].y = leg.properties.hipP[0].y + side * leg.properties.hipLengths[0] * cos(leg.properties.hipSigma[0]);
  leg.properties.hipP[1].z = leg.properties.hipP[0].z - leg.properties.hipLengths[0] * sin(leg.properties.hipSigma[0]);

  leg.properties.hipP[2].y = leg.properties.hipP[1].y + side * leg.properties.hipLengths[1] * cos(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])));
  leg.properties.hipP[2].z = leg.properties.hipP[1].z - leg.properties.hipLengths[1] * sin(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])));

  leg.properties.hipP[5].y = leg.properties.hipP[1].y + side * leg.properties.hipLengths[5] * cos(leg.properties.hipSigma[0] - (M_PI - leg.properties.hipSigma[1]));
  leg.properties.hipP[5].z = leg.properties.hipP[1].z - leg.properties.hipLengths[5] * sin(leg.properties.hipSigma[0] - (M_PI - leg.properties.hipSigma[1]));

  double lr0 = distance(leg.properties.hipP[2], leg.properties.hipP[5]);

  if (side * foot.y < leg.properties.hipP[5].y) { // Left of P4 on left leg, right side of P4 on right leg
    leg.properties.swayAngle = M_PI - fabs(atan((foot.z - leg.properties.hipP[5].z) / (foot.y - leg.properties.hipP[5].y)));
  } else {
    leg.properties.swayAngle = fabs(atan((foot.z - leg.properties.hipP[5].z) / (foot.y - leg.properties.hipP[5].y)));
  }

  leg.properties.hipP[6].y = leg.properties.hipP[5].y + side * leg.properties.hipLengths[6] * cos(leg.properties.swayAngle);
  leg.properties.hipP[6].z = leg.properties.hipP[5].z - leg.properties.hipLengths[6] * sin(leg.properties.swayAngle);

  leg.properties.hipP[4].y = leg.properties.hipP[5].y + side * leg.properties.hipLengths[4] * cos(leg.properties.swayAngle + leg.properties.hipSigma[3]);
  leg.properties.hipP[4].z = leg.properties.hipP[5].z - leg.properties.hipLengths[4] * sin(leg.properties.swayAngle + leg.properties.hipSigma[3]);

  leg.properties.deltaZ = distance(foot, leg.properties.hipP[6]);

  double lr1 = distance(leg.properties.hipP[4], leg.properties.hipP[2]);

  double alpha1 = angleCosineRule(leg.properties.hipLengths[2], leg.properties.hipLengths[3], lr1);
  double alpha2 = angleCosineRule(lr0, lr1, leg.properties.hipLengths[4]);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(leg.properties.hipLengths[3], leg.properties.hipLengths[2], lr1);
  double beta2 = angleCosineRule(leg.properties.hipLengths[4], lr1, lr0);
  double beta = beta1 + beta2;

  double gamma1 = angleCosineRule(lr1, leg.properties.hipLengths[3], leg.properties.hipLengths[2]); // Or M_PI - (alpah1 + beta1)
  double gamma2 = angleCosineRule(lr1, leg.properties.hipLengths[4], lr0); // Or M_PI - (alpah2 + beta2)

  leg.properties.hipP[3].y = leg.properties.hipP[4].y + side * leg.properties.hipLengths[3] * cos(leg.properties.swayAngle + leg.properties.hipSigma[3] + (M_PI - alpha));
  leg.properties.hipP[3].z = leg.properties.hipP[4].z - leg.properties.hipLengths[3] * sin(leg.properties.swayAngle + leg.properties.hipSigma[3] + (M_PI - alpha));

  leg.properties.q0 = -side * atan((leg.properties.hipP[3].z - leg.properties.hipP[2].z) / (leg.properties.hipP[3].y - leg.properties.hipP[2].y));
}

void mainIkine(Leg& leg, coord foot)  {
  int side;
  if (leg.properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  leg.properties.mainP[0] = leg.properties.hipP[6];


  leg.properties.mainP[4] = foot;

  double lr1 = distance(leg.properties.mainP[0], leg.properties.mainP[4]);

  double gamma1 = angleCosineRule(lr1, leg.properties.mainLengths[1], leg.properties.mainLengths[3] + leg.properties.mainLengths[4]);

  double lr2 = sideCosineRule(leg.properties.mainLengths[1], leg.properties.mainLengths[3], gamma1);

  double alpha1 = angleCosineRule(leg.properties.mainLengths[3], leg.properties.mainLengths[1], lr2);
  double alpha2 = angleCosineRule(leg.properties.mainLengths[2], lr2, leg.properties.mainLengths[0]);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(leg.properties.mainLengths[1], leg.properties.mainLengths[3], lr2);
  double beta2 = angleCosineRule(leg.properties.mainLengths[0], lr2, leg.properties.mainLengths[2]);
  double beta = beta1 + beta2;

  double gamma2 = angleCosineRule(lr2, leg.properties.mainLengths[0], leg.properties.mainLengths[2]); // or M_PI - (alpha2 + beta2)

  double theta;
  if (foot.x < leg.properties.hipP[5].x) { // Foot behind P0
    theta = M_PI - fabs(atan((leg.properties.mainP[4].z - leg.properties.mainP[0].z) / (leg.properties.mainP[4].x - leg.properties.mainP[0].x)));
  } else {
    theta = fabs(atan((leg.properties.mainP[4].z - leg.properties.mainP[0].z) / (leg.properties.mainP[4].x - leg.properties.mainP[0].x)));
  }

  double iota = angleCosineRule(leg.properties.mainLengths[3] + leg.properties.mainLengths[4], leg.properties.mainLengths[1], lr1);

  leg.properties.q2 = theta + iota;
  leg.properties.q1 = leg.properties.q2 - alpha;
}

void hipFkine(Leg& leg, double q) {
  int side;
  if (leg.properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  leg.properties.hipP[1].y = leg.properties.hipP[0].y + side * leg.properties.hipLengths[0] * cos(leg.properties.hipSigma[0]);
  leg.properties.hipP[1].z = leg.properties.hipP[0].z - leg.properties.hipLengths[0] * sin(leg.properties.hipSigma[0]);

  leg.properties.hipP[2].y = leg.properties.hipP[1].y + side * leg.properties.hipLengths[1] * cos(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])));
  leg.properties.hipP[2].z = leg.properties.hipP[1].z - leg.properties.hipLengths[1] * sin(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])));

  leg.properties.hipP[5].y = leg.properties.hipP[1].y + side * leg.properties.hipLengths[5] * cos(leg.properties.hipSigma[0] - (M_PI - leg.properties.hipSigma[1]));
  leg.properties.hipP[5].z = leg.properties.hipP[1].z - leg.properties.hipLengths[5] * sin(leg.properties.hipSigma[0] - (M_PI - leg.properties.hipSigma[1]));

  double lr0 = distance(leg.properties.hipP[2], leg.properties.hipP[5]);

  double gamma1 = leg.properties.hipSigma[7] + q;

  leg.properties.hipP[3].y = leg.properties.hipP[2].y + side * leg.properties.hipLengths[2] * cos(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])) - (M_PI - (leg.properties.hipSigma[5] + gamma1)));
  leg.properties.hipP[3].z = leg.properties.hipP[2].z - leg.properties.hipLengths[2] * sin(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])) - (M_PI - (leg.properties.hipSigma[5] + gamma1)));

  double lr1 = distance(leg.properties.hipP[3], leg.properties.hipP[5]);

  double alpha1 = angleCosineRule(lr0, leg.properties.hipLengths[2], lr1);
  double alpha2 = angleCosineRule(leg.properties.hipLengths[4], leg.properties.hipLengths[3], lr1);
  double alpha = alpha1 + alpha2;

  double beta1 = angleCosineRule(leg.properties.hipLengths[2], lr1, lr0);
  double beta2 = angleCosineRule(leg.properties.hipLengths[3], leg.properties.hipLengths[4], lr1);
  double beta = beta1 + beta2;

  leg.properties.hipP[4].y = leg.properties.hipP[3].y + side * leg.properties.hipLengths[3] * cos(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])) - (M_PI - (leg.properties.hipSigma[5] + gamma1)) - (M_PI - alpha));
  leg.properties.hipP[4].z = leg.properties.hipP[3].z - leg.properties.hipLengths[3] * sin(leg.properties.hipSigma[0] - (M_PI - (leg.properties.hipSigma[2] + leg.properties.hipSigma[1])) - (M_PI - (leg.properties.hipSigma[5] + gamma1)) - (M_PI - alpha));

  leg.properties.swayAngle = 2 * M_PI - (leg.properties.hipSigma[3] + leg.properties.hipSigma[6] + leg.properties.hipSigma[8] + beta);

  leg.properties.hipP[6].y = leg.properties.hipP[5].y + side * leg.properties.hipLengths[6] * cos(leg.properties.swayAngle);
  leg.properties.hipP[6].z = leg.properties.hipP[5].z - leg.properties.hipLengths[6] * sin(leg.properties.swayAngle);
}

void mainFkine(Leg& leg, double q1, double q2) {
  int side;
  if (leg.properties.side == 0) {
    side = -1;
  } else {
    side = 1;
  }

  leg.properties.mainP[0] = leg.properties.hipP[6];

  leg.properties.mainP[1].x = leg.properties.mainP[0].x + leg.properties.mainLengths[0] * cos(q1);
  leg.properties.mainP[1].z = leg.properties.mainP[0].z - leg.properties.mainLengths[0] * sin(q1);

  leg.properties.mainP[2].x = leg.properties.mainP[0].x + leg.properties.mainLengths[1] * cos(q2);
  leg.properties.mainP[2].z = leg.properties.mainP[0].z - leg.properties.mainLengths[1] * sin(q2);

  double lr1 = distance(leg.properties.mainP[2], leg.properties.mainP[1]);

  double alpha1 = angleCosineRule(leg.properties.mainLengths[0], leg.properties.mainLengths[1], lr1);
  double alpha2 = angleCosineRule(leg.properties.mainLengths[2], lr1, leg.properties.mainLengths[3]);
  double alpha = alpha1 + alpha2;

  double gamma1 = angleCosineRule(lr1, leg.properties.mainLengths[1], leg.properties.mainLengths[0]); // or q2 - q1
  double gamma2 = angleCosineRule(lr1, leg.properties.mainLengths[3], leg.properties.mainLengths[2]);

  double beta1 = angleCosineRule(leg.properties.mainLengths[1], lr1, leg.properties.mainLengths[0]); // or M_PI - (alpha1 + gamma1)
  double beta2 = angleCosineRule(leg.properties.mainLengths[3], lr1, leg.properties.mainLengths[2]); // or M_PI - (alpha2 + gamma2)
  double beta = beta1 + beta2;

  leg.properties.mainP[3].x = leg.properties.mainP[2].x + leg.properties.mainLengths[3] * cos(q2 - (M_PI - alpha));
  leg.properties.mainP[3].z = leg.properties.mainP[2].z - leg.properties.mainLengths[3] * sin(q2 - (M_PI - alpha));
  // COMPARE THESE TO AVOID IMPOSSIBLE JOINT ANGLES
  leg.properties.mainP[3].x = leg.properties.mainP[1].x + leg.properties.mainLengths[2] * cos(q1 + (M_PI - beta));
  leg.properties.mainP[3].z = leg.properties.mainP[1].z - leg.properties.mainLengths[2] * sin(q1 + (M_PI - beta));

  leg.properties.mainP[4].x = leg.properties.mainP[2].x + (leg.properties.mainLengths[3] + leg.properties.mainLengths[4]) * cos(q2 - (M_PI - alpha));
  leg.properties.mainP[4].z = leg.properties.mainP[2].z - (leg.properties.mainLengths[3] + leg.properties.mainLengths[4]) * sin(q2 - (M_PI - alpha));

  leg.properties.deltaX = leg.properties.mainP[4].x - leg.properties.mainP[0].x;
  leg.properties.deltaZ = leg.properties.mainP[0].z - leg.properties.mainP[4].z;

  //Calculate y of each point based on leg sway and update z of each point to accomodate this sway angle
  double deltaZ1 = leg.properties.mainP[0].z - leg.properties.mainP[1].z;
  leg.properties.mainP[1].y = leg.properties.mainP[0].y + side * deltaZ1 * cos(leg.properties.swayAngle);
  leg.properties.mainP[1].z = leg.properties.mainP[0].z - deltaZ1 * sin(leg.properties.swayAngle);

  double deltaZ2 = leg.properties.mainP[0].z - leg.properties.mainP[2].z;
  leg.properties.mainP[2].y = leg.properties.mainP[0].y + side * deltaZ2 * cos(leg.properties.swayAngle);
  leg.properties.mainP[2].z = leg.properties.mainP[0].z - deltaZ2 * sin(leg.properties.swayAngle);

  double deltaZ3 = leg.properties.mainP[0].z - leg.properties.mainP[3].z;
  leg.properties.mainP[3].y = leg.properties.mainP[0].y + side * deltaZ3 * cos(leg.properties.swayAngle);
  leg.properties.mainP[3].z = leg.properties.mainP[0].z - deltaZ3 * sin(leg.properties.swayAngle);

  leg.properties.mainP[4].y = leg.properties.mainP[0].y + side * leg.properties.deltaZ * cos(leg.properties.swayAngle);
  leg.properties.mainP[4].z = leg.properties.mainP[0].z - leg.properties.deltaZ * sin(leg.properties.swayAngle);
}




void moveL(Leg leg, coord goalPose) { //Linear move foot

}

void triggL(Leg leg, coord goalPose) {

}

void mapInputs() {

}

void mapOutputs() {

}

void setMaxFootprint(Leg leg) {

}
