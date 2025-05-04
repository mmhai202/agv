#pragma once
#include "Config.h"
#include <Arduino.h>

class Encoder {
public:
  void begin();
  void start();
  void stop();
  void reset();
  int getL();
  int getR();
private:
  static void IRAM_ATTR isrLeft();
  static void IRAM_ATTR isrRight();
  static volatile int encoderL;
  static volatile int encoderR;
  static volatile bool risingLeftDetected;
  static volatile bool risingRightDetected;
  static unsigned long lastDebounceLeft;
  static unsigned long lastDebounceRight;
  static unsigned long debounceDelay;
};
