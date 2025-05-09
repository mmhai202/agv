#pragma once
#include <Arduino.h>
#include "Config.h"

class Uart2 {
public:
  void begin(HardwareSerial* port, uint32_t baud = 115200, int8_t rx=16, int8_t tx=17);
  bool available();
  Data read();
  int id;
  int angle;
private:
  HardwareSerial* _ser;
};