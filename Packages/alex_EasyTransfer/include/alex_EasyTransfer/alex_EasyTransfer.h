#ifndef EasyTransfer_h
#define EasyTransfer_h

#define details(name) (byte*)&name, sizeof(name)

class EasyTransfer {
public:
  void begin(uint8_t *, uint8_t, char * port);
  void sendData();
  bool receiveData();

private:
  char * _port;
  uint8_t * address;
  uint8_t size;
  uint8_t * rx_buffer;
  uint8_t rx_array_inx;
  uint8_t rx_len;
  uint8_t calc_CS;
};

#endif
