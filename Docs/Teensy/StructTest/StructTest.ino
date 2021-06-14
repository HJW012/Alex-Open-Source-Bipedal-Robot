#include <AlexLib.h>
//
//#define DEBUG_SERIAL Serial
//#define DEBUG(x) Serial.println(x)
//const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

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

#define xIncrement 1
#define yIncrement 1
#define zIncrement 1


float p_out;
float v_out;
float t_out;

Alex alex;

class joystick {
  private:

  public:
    bool debounce = false;
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
        //debounce = true;
      } else if (digitalRead(buttonPin)) {
        buttonState = false;
        //debounce = false;
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

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);


void setup() {
  setMotorIDs();
  alex.dxl = dxl;
  alex.init();

  setConstantCoords();

  while (!DEBUG_SERIAL);
  DEBUG("Waiting for power");
  //while (!alex.status.powerStatus);
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



  leftJoystick.init();
  leftJoystick.setCentre(624, 563, 571);
  rightJoystick.init();
  rightJoystick.setCentre(556, 540, 543);


  pinMode(zeroMotorButton, INPUT_PULLUP);
  pinMode(motorModeButton, INPUT_PULLUP);
  //dxl.setOperatingMode(leg_1_hip_motor_ID, OP_POSITION);
  //dxl.writeControlTableItem(PROFILE_VELOCITY, leg_1_hip_motor_ID, 30);
  pinMode(DXL_DIR_PIN, OUTPUT);
  digitalWrite(DXL_DIR_PIN, HIGH);
}

void loop() {
  if (alex.status.fs) {

  } else {

  }

  alex.ES_check();
  alex.sendReceiveCycle();
  alex.convertJointAngles();
  alex.updateJointStatus();
  alex.calcJointAngles();
  Serial.println(alex.leg[0].hip_motor.out.p_out);
  Serial.println(alex.leg[1].hip_motor.out.p_out);
  alex.updatePoseStatus();
  alex.poseCheck();

  handleButtons();
  handleJoysticks();


  //DEBUG("Left q0: " + String(alex.leg[0].properties.current_q0));
  //  DEBUG("Left q1: " + String(rad2deg(alex.leg[0].properties.current_q1)));
  //  DEBUG("Left q2: " + String(rad2deg(alex.leg[0].properties.current_q2)));
  //  DEBUG("Left Calculated Pose: " + String(alex.leg[0].properties.current_foot_pose.x) + ", " + String(alex.leg[0].properties.current_foot_pose.y) + ", " + String(alex.leg[0].properties.current_foot_pose.z));
  //alex.leg[1].ikine(alex.leg[1].properties.current_foot_pose);
  //alex.leg[0].ikine(alex.leg[0].properties.current_foot_pose);
  //  DEBUG("Left goal pose x: " + String(alex.leg[0].properties.current_foot_pose.x));
  //  DEBUG("Left goal pose y: " + String(alex.leg[0].properties.current_foot_pose.y));
  //  DEBUG("Left goal pose z: " + String(alex.leg[0].properties.current_foot_pose.z));
  //DEBUG("Right Current Pose: " + String(alex.leg[1].properties.current_foot_pose.x) + ", " + String(alex.leg[1].properties.current_foot_pose.y) + ", " + String(alex.leg[1].properties.current_foot_pose.z));
  //  DEBUG("Left Calculated q0: " + String(alex.leg[0].properties.calculated_q0));
  //  //DEBUG("Right Calculated q0: " + String(alex.leg[1].properties.calculated_q0));
  //  DEBUG("Left Calculated q1: " + String(rad2deg(alex.leg[0].properties.calculated_q1)));
  //  DEBUG("Left Calculated q2: " + String(rad2deg(alex.leg[0].properties.calculated_q2)));
  ////  DEBUG("Left Goal q0: " + String(alex.leg[0].properties.goal_q0));
  //DEBUG("Left Goal q1: " + String(rad2deg(alex.leg[0].properties.goal_q1)));
  //DEBUG("Left Goal q2: " + String(rad2deg(alex.leg[0].properties.goal_q2)));
  //  DEBUG("Left Tracking: " + String(alex.leg[0].properties.trackGoalFootPose));
  ////  DEBUG("Right Tracking: " + String(alex.leg[1].properties.trackGoalFootPose));
  //  DEBUG("Left Current Pose: " + String(alex.leg[0].properties.current_foot_pose.x) + ", " + String(alex.leg[0].properties.current_foot_pose.y) + ", " + String(alex.leg[0].properties.current_foot_pose.z));
  //  DEBUG("Left Goal Pose: " + String(alex.leg[0].properties.goal_foot_pose.x) + ", " + String(alex.leg[0].properties.goal_foot_pose.y) + ", " + String(alex.leg[0].properties.goal_foot_pose.z));
  //  DEBUG("In Motor Mode: " + String(alex.status.inMotorMode));
  //  DEBUG("p_in 0: " + String(alex.leg[1].hip_motor.in.p_in));
  //  DEBUG("p_in 1: " + String(alex.leg[1].knee_motor_1.in.p_in));
  //  DEBUG("p_in 2: " + String(alex.leg[1].knee_motor_2.in.p_in));
  //  DEBUG("p_out 0: " + String(alex.leg[1].hip_motor.out.p_out));
  //  DEBUG("p_out 1: " + String(alex.leg[1].knee_motor_1.out.p_out));
  //  DEBUG("p_out 2: " + String(alex.leg[1].knee_motor_2.out.p_out));
  //  DEBUG("v_out 0: " + String(alex.leg[0].knee_motor_1.out.v_out));
  DEBUG("Left p_in 1: " + String(alex.leg[0].knee_motor_1.in.p_in));
  DEBUG("left p_in 2: " + String(alex.leg[0].knee_motor_2.in.p_in));
  DEBUG("Left p_out 1: " + String(alex.leg[0].knee_motor_1.out.p_out));
  DEBUG("Left p_out 2: " + String(alex.leg[0].knee_motor_2.out.p_out));
  DEBUG("Left q1: " + String(rad2deg(alex.leg[0].properties.current_q1)));
  DEBUG("Left q2: " + String(rad2deg(alex.leg[0].properties.current_q2)));
  DEBUG("Left Goal Pose: " + String(alex.leg[0].properties.goal_foot_pose.x) + ", " + String(alex.leg[0].properties.goal_foot_pose.y) + ", " + String(alex.leg[0].properties.goal_foot_pose.z));

  DEBUG("Right p_in 1: " + String(alex.leg[1].knee_motor_1.in.p_in));
  DEBUG("Right p_in 2: " + String(alex.leg[1].knee_motor_2.in.p_in));
  DEBUG("Right p_out 1: " + String(alex.leg[1].knee_motor_1.out.p_out));
  DEBUG("Right p_out 2: " + String(alex.leg[1].knee_motor_2.out.p_out));
  DEBUG("Right q1: " + String(rad2deg(alex.leg[1].properties.current_q1)));
  DEBUG("Right q2: " + String(rad2deg(alex.leg[1].properties.current_q2)));
  DEBUG("Right Goal Pose: " + String(alex.leg[1].properties.goal_foot_pose.x) + ", " + String(alex.leg[1].properties.goal_foot_pose.y) + ", " + String(alex.leg[1].properties.goal_foot_pose.z));


  // Perform some action based on input
  // Send data to motors
  delayMicroseconds(500);
}

void setInitialPoses() {
  alex.leg[0].hipFkine(deg2rad(15));
  alex.leg[0].mainFkine(deg2rad(23.698), deg2rad(142.333));
}

void setMotorIDs() {
  alex.leg[0].hip_motor.ID = leg_1_hip_motor_ID;
  alex.leg[0].knee_motor_1.ID = leg_1_knee_motor_1_ID;
  alex.leg[0].knee_motor_2.ID = leg_1_knee_motor_2_ID;

  alex.leg[1].hip_motor.ID = leg_2_hip_motor_ID;
  alex.leg[1].knee_motor_1.ID = leg_2_knee_motor_1_ID;
  alex.leg[1].knee_motor_2.ID = leg_2_knee_motor_2_ID;
}

void setConstantCoords() {
  alex.properties.p[0] = coord(0, 0, 0); // Centre
  alex.properties.p[1] = coord(0, 32.497, -26.038); // Left Hip Connection
  alex.properties.p[2] = coord(0, -32.497, -26.038); // Right Hip Connection

  alex.leg[0].properties.hipP[0] = alex.properties.p[1];
  alex.leg[1].properties.hipP[0] = alex.properties.p[2];
  alex.leg[0].properties.side = 0;
  alex.leg[1].properties.side = 1;
}

void handleJoysticks() {
  leftJoystick.getData();
  rightJoystick.getData();
  if (leftJoystick.buttonState && !leftJoystick.debounce) {
    alex.leg[0].toggleGoalTracking();
    leftJoystick.debounce = true;
  } else if (!leftJoystick.buttonState) {
    leftJoystick.debounce = false;
  }

  if (rightJoystick.buttonState && !rightJoystick.debounce) {
    alex.leg[1].toggleGoalTracking();
    rightJoystick.debounce = true;
  } else if (!rightJoystick.buttonState) {
    rightJoystick.debounce = false;
  }

  //
  // LEFT
  if (abs(leftJoystick.xVal - leftJoystick.centreX) >= leftJoystick.increment) {
    int multiplyer = (leftJoystick.xVal - leftJoystick.centreX) / leftJoystick.increment;
    alex.leg[0].properties.goal_foot_pose.y -= (yIncrement * multiplyer);
  }

  if (abs(leftJoystick.yVal - leftJoystick.centreY) >= leftJoystick.increment) {
    int multiplyer = (leftJoystick.yVal - leftJoystick.centreY) / leftJoystick.increment;
    alex.leg[0].properties.goal_foot_pose.x += (xIncrement * multiplyer);

  }

  if (abs(leftJoystick.zVal - leftJoystick.centreZ) >= leftJoystick.increment) {
    int multiplyer = (leftJoystick.zVal - leftJoystick.centreZ) / leftJoystick.increment;
    alex.leg[0].properties.goal_foot_pose.z -= (zIncrement * multiplyer);
  }
  //
  // RIGHT
  if (abs(rightJoystick.xVal - rightJoystick.centreX) >= rightJoystick.increment) {
    int multiplyer = (rightJoystick.xVal - rightJoystick.centreX) / rightJoystick.increment;
    alex.leg[1].properties.goal_foot_pose.y -= (yIncrement * multiplyer);
  }

  if (abs(rightJoystick.yVal - rightJoystick.centreY) >= rightJoystick.increment) {
    int multiplyer = (rightJoystick.yVal - rightJoystick.centreY) / rightJoystick.increment;
    alex.leg[1].properties.goal_foot_pose.x += (xIncrement * multiplyer);
  }

  if (abs(rightJoystick.zVal - rightJoystick.centreZ) >= rightJoystick.increment) {
    int multiplyer = (rightJoystick.zVal - rightJoystick.centreZ) / rightJoystick.increment;
    alex.leg[1].properties.goal_foot_pose.z -= (zIncrement * multiplyer);
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
    if (!alex.status.inMotorMode) {
      alex.motorsOn();
    } else {
      alex.motorsOff();
    }
  } else if (digitalRead(motorModeButton)) {
    debounce2 = false;
  }
}

void zeroMotors() {
  // Leg
  delay(500);
  DEBUG("Move leg 1 knee motor 1 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[0].knee_motor_1.zero();
  delay(500);
  DEBUG("Move leg 1 knee motor 2 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[0].knee_motor_2.zero();
  delay(500);
  DEBUG("Move leg 1 hip motor to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[0].hip_motor.zero();
  delay(500);

  // Right
  DEBUG("Move leg 2 knee motor 1 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[1].knee_motor_1.zero();
  delay(500);
  DEBUG("Move leg 2 knee motor 2 to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[1].knee_motor_2.zero();
  delay(500);
  DEBUG("Move leg 2 hip motor to zero position then press the zero motor button.");
  while (digitalRead(zeroMotorButton));
  alex.leg[1].hip_motor.zero();
  delay(500);
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
