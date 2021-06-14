#include <EasyTransfer.h>


struct data_structure {

};

class Device {
  protected:
    EasyTransfer _ET;
    data_structure _data;
    int _id = 0;

  public:
    bool connected = false;

    Device() {

    }

    void setID(int ID) {
      _id = ID;
    }

    int getID() {
      return _id;
    }

    void setET(EasyTransfer & ET) {
      _ET = ET;
    }

    void setData(data_structure & data) {
      _data = data;
    }
};

class Master : public Device {
  private:

  public:
    int connectedSlaves[10];
    int slaveIndex = 0;
    Master() {

    }

    void setup(data_structure & data, Stream & stream, EasyTransfer & ET) {
      setET(ET);
      _ET.begin(details(_data), &stream);
    }

    void establishConnection(int ID) {
      connectedSlaves[slaveIndex] = ID;
      slaveIndex++;

      
    }
};

/*

  // Only 1 master in network
  class master {
  private:
    Stream & _stream;

  public:
    EasyTransfer & network;

    master() {
      ET.begin(details(data), &Serial1);
    }

    // Set serial interface
    void setStream(Stream & stream) {
      _stream = stream;
    }

    // Connect to the slave ID in param
    void connect(int ID) {
      if (!connected && !data.commsEstablished) {
        if (ET.receiveData() && ) {

        }
      }
    }

    // Maintain connection with slave ID in param
    bool maintainConnection(int ID) {

    }
  };

  // Multiple slaves in network
  class slave {
  private:

  public:
    slave() {

    }
  };

*/

data_structure data;
EasyTransfer ET;
Master master;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  master.setID(0);
  master.setup(data, Serial1, ET);



  master.establishConnection(1);
}

void loop() {
  // put your main code here, to run repeatedly:

}
