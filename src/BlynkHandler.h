#pragma once
#include "Vehicle.h"

class BlynkHandler {
public:
  void begin(Vehicle* v);
  void run();
  void write(int pin, int value);
  static Vehicle* vehicle;
};