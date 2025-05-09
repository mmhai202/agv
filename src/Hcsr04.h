#pragma once
#include <Arduino.h>
#include "Config.h"

class hcsr04 {
public:
  void begin();
  void run();
  int distance1=0;
  int distance2=0;
private:
};