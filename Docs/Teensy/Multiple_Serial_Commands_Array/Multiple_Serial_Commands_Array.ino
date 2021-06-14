#include <AccelStepper.h>
AccelStepper stepper(AccelStepper::DRIVER, 3, 4);

String functions[] = {"<MOV>", "<JOG>"};
String commands[] = {"STOP", "STEP", "REPEAT", "REVERSE", "HELP", "?"};
String input;

bool commandPresent = false;
bool functionPresent = false;

String finalInput;

int commaLocations[5];

String steps = "";
String maxVel = "";
String accel = "";
String decel;

bool go = false;

bool pressed = false;
long prevPress = 0;
long curPress;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(6, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
   // if (!stepper.isRunning()) {
      
      input = Serial.readStringUntil("\n");

      if (commandCheck(input)) {

      } else {
        error();
      }


    //}
  } else {
    /*go = false;
    stepper.stop();*/
  }
  if (go) {
    stepper.run();
    if (finalInput == "<JOG>" && stepper.distanceToGo() < 10000) {
      stepper.move(10000);
    }
  } else {
    stepper.run();
  }
curPress = millis();
  if (!digitalRead(6) && curPress - prevPress > 100) {
    Serial.println("STOPPING NOW");
    go = false;
    stepper.stop();
    prevPress = millis();
  } else {
  }
}

bool commandCheck(String commandEntered) {
  int commaCount = 0;
  String tempCommandClose;
  steps = "";
  maxVel = "";
  accel = "";
  functionPresent = false;
  commandPresent = false;

  for (auto x : functions) {
    tempCommandClose = "";
    for (auto y : x) {
      tempCommandClose += y;
      if (y == '<') {
        tempCommandClose += "/";
      }
    }

    int openIndex = commandEntered.indexOf(x);
    int openRepeatIndex = commandEntered.lastIndexOf(x);
    int closeIndex = commandEntered.indexOf(tempCommandClose);
    int closeRepeatIndex = commandEntered.lastIndexOf(tempCommandClose);


    if (openIndex == 0 && closeIndex > openIndex && openIndex == openRepeatIndex && closeIndex == closeRepeatIndex && !functionPresent && !commandPresent) {
      finalInput = x;
      functionPresent = true;
    }
  }

  for (auto x : commands) {
    int commandIndex = commandEntered.indexOf(x);
    int commandRepeatIndex = commandEntered.lastIndexOf(x);
    if (commandIndex == 0 && x.length() == commandEntered.length() - 1 && commandIndex == commandRepeatIndex && !functionPresent && !commandPresent) {
      finalInput = x;
      commandPresent = true;
    }
  }

  if (functionPresent && commandPresent || !functionPresent && !commandPresent) {
    return false;
  }


  if (functionPresent) {
    if (!stepper.isRunning()) {
      for (int i = 0; i < commandEntered.length(); i++) {
        if (commandEntered[i] == ',') {
          commaLocations[commaCount] = i;
          commaCount ++;
        }
      }
      if (finalInput == "<MOV>") {
        if (commaCount != 2) {
          return false;
        }

        for (int i = finalInput.length(); i < commaLocations[0]; i++) {
          if (!isdigit(commandEntered[i])) {
            return false;
          }
          steps += commandEntered[i];
        }
        for (int i = commaLocations[0] + 1; i < commaLocations[1]; i++) {
          if (!isdigit(commandEntered[i])) {
            return false;
          }
          maxVel += commandEntered[i];
        }
        for (int i = commaLocations[1] + 1; i < (commandEntered.length() - tempCommandClose.length() - 1); i++) {
          if (!isdigit(commandEntered[i])) {
            return false;
          }
          accel += commandEntered[i];
        }
        Serial.println("MOVE COMMAND");
        Serial.println(steps);
        Serial.println(maxVel);
        Serial.println(accel);
        stepper.setCurrentPosition(0);
        stepper.setMaxSpeed(maxVel.toInt());
        stepper.setAcceleration(accel.toInt());
        stepper.moveTo(steps.toInt());
        go = true;
        stepper.run();
      } else if (finalInput == "<JOG>") {
        if (commaCount != 1) {
          return false;
        }
        for (int i = finalInput.length(); i < commaLocations[0]; i++) {
          if (!isdigit(commandEntered[i])) {
            return false;
          }
          maxVel += commandEntered[i];
        }
        for (int i = commaLocations[0] + 1; i < (commandEntered.length() - tempCommandClose.length() - 1); i++) {
          if (!isdigit(commandEntered[i])) {
            return false;
          }
          accel += commandEntered[i];
        }
        Serial.println("JOG COMMAND");
        Serial.println(maxVel);
        Serial.println(accel);
        stepper.setCurrentPosition(0);
        stepper.setMaxSpeed(maxVel.toInt());
        stepper.setAcceleration(accel.toInt());
        stepper.move(10000);
        go = true;
        stepper.run();
      }
    }
  } else {
    if (finalInput == "STOP") {
      Serial.println("STOPPING");
      go = false;
      stepper.stop();
    } else if (finalInput == "STEP") {
      Serial.println("MOVING ONE STEP");
      stepper.setCurrentPosition(0);
      stepper.moveTo(1);
    } else if (finalInput == "REVERSE") {
      Serial.println("REPEATING LAST COMMAND IN REVERSE DIRECTION");
    } else if (finalInput == "REPEAT") {
      Serial.println("REPEATING LAST COMMAND");
    } else if (finalInput == "HELP" || finalInput == "?") {
      Serial.println("HELP MEH");
    }
  }

  return true;
}

void handleCommands() {

}

void error() {
  Serial.println("Ërror");
}
