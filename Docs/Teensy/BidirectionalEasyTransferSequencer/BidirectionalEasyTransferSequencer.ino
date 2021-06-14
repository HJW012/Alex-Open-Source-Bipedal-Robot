#include <EasyTransfer.h>

#define ID 0
#define loopDelay 1 // ms - delay between each loop
#define commsInitFrequency 2 // Hz - frequency of attempts to connect with slave
#define commsInitInterval 1000 / commsInitFrequency // ms - time between attempts to inti comms with slave
#define receiveDataDelay 1 // ms - delay after sending data and checkign for receiving data
#define slaveID 1
#define transferFrequency 50
#define transferInterval 1000 / transferFrequency
#define receiveTimeout 200
#define receiveTimeoutTries 3 // Amount of retries
#define duplexPin 17 // Pins used to control data flow of Max485 - off on receive, on for transmit

EasyTransfer network;
struct data_structure {
  // COMMS
  int dev_ID = 0;
  int target_dev_ID = 1;
  bool heartbeat = true;
  bool initComms = true;
  bool ack;
  bool nack;
  int errorCode;
  int timestamp;

  // DATA
  int testInt = 1;
};
data_structure data;
bool commsEstablished = false;
bool transferCycle = 0; // 1 is transmit, 0 is receive
bool dataSent = false;
bool prevHeartbeat = false;

int sequence_step = 0;

int prevInitTime = 0;

int receiveTimeoutCount = 0;

elapsedMillis initTime;
elapsedMillis transferTime;
elapsedMillis receiveTime;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  network.begin(details(data), &Serial1);

  pinMode(13, OUTPUT);
  pinMode(duplexPin, OUTPUT);
  digitalWrite(duplexPin, HIGH);
}

void loop() {
  debug("Test int: " + String(data.testInt));
  sequencer(sequence_step);

  delay(loopDelay);
}

void sequencer(int step) {
  /* MASTER SEQUENCER BREAKDOWN */

  //  sequence step 0: Init Comms
  //    Stay in 0 until connected
  //    Move to 1 once connected

  //  sequence step 1: Send data
  //    Move to 2 once data sent

  //  sequence step 2: Receive data
  //    Move to 3 once data received
  //    Move to 7 upon receive timeout

  //  sequence step 3: Maintain Connection
  //    Move to 4 if connection healthy
  //    Move to 6 if heartbeat unhealthy

  //  sequence step 4: Calculations
  //    Move to 5 once calculated

  //  sequence step 5: Manipulate data
  //    Move to 1 once data manipulated

  //  sequence step 6: Heartbeat Unhealthy
  //    Move to 8 if heartbeat unhealthy

  //  sequence step 7: Receive timeout
  //    Move to 2 for receive fail retries
  //    Move to 8 after fail retries

  //  sequence step 8: Connection lost
  //    Move to 9 if connection lost due to heartbeat, receive timeout, or other

  //  sequence step 9: Reconnect
  //    Move to 0 if needing to reconnect

  debug("Step " + String(step));
  if (transferTime >= transferInterval) {
    transferCycle = !transferCycle;
    dataSent = false;
    transferTime = 0;
  }

  /*if (transferCycle) {
    digitalWrite(duplexPin, HIGH);
  } else {
    digitalWrite(duplexPin, LOW);
  }*/

  switch (step) {
    case 0: //
      digitalWrite(13, LOW);
      step_0();
      break;
    case 1: //
      digitalWrite(13, HIGH);
      step_1();
      break;
    case 2: //
      step_2();
      break;
    case 3: //
      step_3();
      break;
    case 4: //
      step_4();
      break;
    case 5: //
      step_5();
      break;
    case 6: //
      digitalWrite(13, LOW);
      step_6();
      break;
    case 7: //
      digitalWrite(13, LOW);
      step_7();
      break;
    case 8: //
      digitalWrite(13, LOW);
      step_8();
      break;
    case 9: //
      step_9();
      break;
    default:
      break;
  }
}

void debug(String msg) {
  Serial.print(millis());
  Serial.print(" - ");
  Serial.print(" DEBUG: ");
  Serial.println(msg);
}

void step_0() {
  digitalWrite(duplexPin, HIGH);
  if (initTime >= commsInitInterval) {
    resetHeartbeat();
    data.initComms = true;
    data.dev_ID = ID;
    data.target_dev_ID = slaveID;
    network.sendData();
    initTime = 0;
  } else {
     digitalWrite(duplexPin, LOW);
    if (initTime >= receiveDataDelay) {
      if (network.receiveData()) {
        if (data.dev_ID == slaveID && data.target_dev_ID == ID && data.initComms) {
          commsEstablished = true;
          data.initComms = false;
          delay(10);
          sequence_step = 1;
          transferTime = 0;
        }
      }
    }
  }
}

void step_1() {
  if (transferCycle && !dataSent) {
    data.dev_ID = ID;
    data.target_dev_ID = slaveID;
    data.timestamp = millis();
    network.sendData();
    debug("Data sent now");
    dataSent = true;
    sequence_step = 2;
    prevHeartbeat = data.heartbeat;
    receiveTime = 0;
    receiveTimeoutCount = 0;

  }
}

void step_2() {
  if (!transferCycle) {
    if (receiveTime >= receiveTimeout) {
      sequence_step = 7;
    }
    if (network.receiveData()) {
      if (data.dev_ID == slaveID && data.target_dev_ID == ID) {
        sequence_step = 3;
      }
    }
  }
}

void step_3() {
  if (data.heartbeat != prevHeartbeat) {
    // Healthy heartbeat
    sequence_step = 4;
    prevHeartbeat = data.heartbeat;
  } else {
    // Unhealthy heartbeat
    sequence_step = 6;
  }
}

void step_4() {
  sequence_step = 5;
}

void step_5() {
  data.testInt = data.testInt * 2;
  sequence_step = 1;
}

void step_6() {
  Serial.println("Heartbeat unhealthy");
  sequence_step = 8;
}

void step_7() {
  if (receiveTimeoutCount < receiveTimeoutTries) {
    Serial.println("Response timeout: retry " + String(receiveTimeoutCount));
    receiveTimeoutCount++;
    sequence_step = 2;
  } else {
    Serial.println("Response timeout: connection lost");
    sequence_step = 8;
  }
}

void step_8() {
  Serial.println("Connection Lost");
  commsEstablished = false;
  sequence_step = 9;
}

void step_9() {
  Serial.println("Attempting to reconnect");
  sequence_step = 0;
}

void resetHeartbeat()  {
  data.heartbeat = true;
  prevHeartbeat = false;
}
