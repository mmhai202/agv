#include "Encoder.h"

volatile int Encoder::encoderL = 0;
volatile int Encoder::encoderR = 0;
volatile bool Encoder::risingLeftDetected = false;
volatile bool Encoder::risingRightDetected = false;
unsigned long Encoder::lastDebounceLeft = 0;
unsigned long Encoder::lastDebounceRight = 0;
unsigned long Encoder::debounceDelay = 500;

void Encoder::begin() {
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
}
void Encoder::start() {
  reset();
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isrRight, RISING);
}
void Encoder::stop() {
  detachInterrupt(digitalPinToInterrupt(ENCODER_L));
  detachInterrupt(digitalPinToInterrupt(ENCODER_R));
}

void Encoder::reset() {encoderL = 0; encoderR = 0;}

int Encoder::getL() {return encoderL;}

int Encoder::getR() {return encoderR;}

void IRAM_ATTR Encoder::isrLeft() {
  unsigned long currentTime = micros();
  if (currentTime - lastDebounceLeft > debounceDelay) {
    encoderL++;
    lastDebounceLeft = currentTime;
  }
}
void IRAM_ATTR Encoder::isrRight() {
  unsigned long currentTime = micros();
  if (currentTime - lastDebounceRight > debounceDelay) {
    encoderR++;
    lastDebounceRight = currentTime;
  }
}