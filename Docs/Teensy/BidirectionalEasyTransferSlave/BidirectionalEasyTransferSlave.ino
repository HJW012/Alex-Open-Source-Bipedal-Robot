#include <EasyTransfer.h>

#define ID 1

struct data_structure {
  bool initComms;
  bool heartbeat;
  bool requestResponse;
};

data_structure data;

bool prevHeartbeat;
EasyTransfer network;

bool role = 1; // Master: 0, Slave: 1

bool commsEstablished = false;
bool prevCommsEstablished = false;
bool dataReceived = false;
bool responseRequested = false;


void setup() {
  Serial.begin(9600); // Debug serial stream
  Serial1.begin(9600);  // EasyTransfer stream

  network.begin(details(data), &Serial1);

  pinMode(13, OUTPUT);
}

// Operation Overview
// 1. Establish connection
// 2. Maintain Connection
// 3. Receive data
// 3. Manipulate data
// 4. Send response

void loop() {
  digitalWrite(13, LOW);

  // 1: If no connection established, wait until connection is established
  while (!commsEstablished) {
    if (!prevCommsEstablished) {
      Serial.println("Attempting to establish connection");
    } else {
      Serial.println("Connection Lost: Attempting to reconnect");
    }
    establishConnection();
  }

  digitalWrite(13, HIGH);

  // 2: Read Data
  if (network.receiveData()) {
    dataReceived = true;
  }

  // 3: Maintain connection
  if (dataReceived) {
    prevHeartbeat = data.heartbeat;
    if (data.requestResponse) {
      responseRequested = true;
      if (data.heartbeat == prevHeartbeat) {
        // Heartbeat healthy
        Serial.println("Heartbeat healthy");
        data.heartbeat = !data.heartbeat;
      } else {
        // Heartbeat unhealthy
        Serial.println("Heartbeat unhealthy");
      }
    }
  }

  // 4: Maniipulate Data

  // 5: Send Response
  if (responseRequested) {
    data.requestResponse = false;
    network.sendData();
  }



  dataReceived = false;
  responseRequested = false;
}

void establishConnection() {
  if (network.receiveData()) {
    if (data.initComms) {
      commsEstablished = true;
      network.sendData();
      Serial.println("Connection Established");
    }
  }
}

// Check heartbeat and continue receiving/transferring data
// Master sends and echos heartbeat from slave, slave inverts heartbeat and sends back
void readData() {
  if (network.receiveData()) {
    prevHeartbeat = data.heartbeat;
    if (data.requestResponse) {
      if (data.heartbeat == prevHeartbeat) {
        // Heartbeat healthy
        Serial.println("Heartbeat healthy");
        data.heartbeat = !data.heartbeat;
      } else {
        // Heartbeat unhealthy
        Serial.println("Heartbeat unhealthy");
      }
    }
  }
}
