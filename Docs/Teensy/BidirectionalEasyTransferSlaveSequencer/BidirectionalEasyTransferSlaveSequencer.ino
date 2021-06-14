#include <EasyTransfer.h>

#define ID 1
#define loopDelay 1
#define masterID 0
#define responseDelay 2 // ms
#define receiveTimeoutTries 3 // Amount of retries
#define receiveTimeout 200 // ms 
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
bool prevHeartbeat = true;

int sequence_step = 0;

int prevInitTime = 0;

int receiveTimeoutCount = 0;

elapsedMillis initTime;
elapsedMillis receiveTime;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  network.begin(details(data), &Serial1);

  pinMode(13, OUTPUT);
  pinMode(duplexPin, OUTPUT);
  digitalWrite(duplexPin, LOW);
}

void loop() {
  sequencer(sequence_step);

  delay(loopDelay);
}

void sequencer(int step) {
  /* SLAVE SEQUENCER BREAKDOWN */

  //  sequence step 0: Init Comms
  //    Stay in 0 until connected
  //    Move to 1 once connected

  //  sequence step 1: Receive data
  //    Move to 2 once data received

  //  sequence step 2: Maintain Connection
  //    Move to 3 if connection healthy
  //    Move to 6 if heartbeat unhealthy

  //  sequence step 3: Calculations
  //    Move to 4 once calculated

  //  sequence step 4: Manipulate data
  //    Move to 5 once data manipulated

  //  sequence step 5: Send data
  //    Move to 1 once data sent

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
  digitalWrite(duplexPin, LOW);
  resetHeartbeat();
  //digitalWrite(duplexPin, LOW);
  if (network.receiveData()) {
    if (data.dev_ID == masterID && data.target_dev_ID == ID && data.initComms) {
      commsEstablished = true;
      initTime = 0;
      //digitalWrite(duplexPin, HIGH);
    }
  }

  if (commsEstablished && initTime >= responseDelay) {
    digitalWrite(duplexPin, HIGH);
    data.dev_ID = ID;
    data.target_dev_ID = masterID;
    Serial.println("Init comms receive");
    data.initComms = true;
    //digitalWrite(duplexPin, HIGH);
    network.sendData();
    sequence_step = 1;
    receiveTime = 0;
    receiveTimeoutCount = 0;
    //digitalWrite(duplexPin, LOW);
  }
}

void step_1() {
  // this line may cause timeout issues
  if (receiveTime >= receiveTimeout) {
    sequence_step = 7;
  }
  
  if (network.receiveData()) {
    if (data.dev_ID == masterID && data.target_dev_ID == ID) {
      receiveTime = 0;
      sequence_step = 2;
    }
  }
}

void step_2() {
  if (data.heartbeat == prevHeartbeat) {
    // Healthy heartbeat
    data.heartbeat = !data.heartbeat;
    prevHeartbeat = data.heartbeat;
    sequence_step = 3;
  } else {
    // Unhealthy heartbeat
    sequence_step = 6;
  }
}

void step_3() {
  sequence_step = 4;
}

void step_4() {
  data.testInt++;
  sequence_step = 5;
}

void step_5() {
  data.dev_ID = ID;
  data.target_dev_ID = masterID;
  delay(responseDelay);
  network.sendData();
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
    receiveTime = 0;
    sequence_step = 1;
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

void resetHeartbeat() {
  prevHeartbeat = true;
}
