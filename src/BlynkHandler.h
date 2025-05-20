#pragma once
#include "Vehicle.h"
#include "Encoder.h"

class BlynkHandler {
public:
  void begin(Vehicle* v, Encoder* e);
  void run();
  void write(int pin, int value);
  static Vehicle* vehicle;
  static Encoder* encoder;
};