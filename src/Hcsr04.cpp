#include "Hcsr04.h"

void hcsr04::begin() {
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
}

void hcsr04::run() {
    long duration;

    // Đo cảm biến 1
    digitalWrite(TRIG1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG1, LOW);

    duration = pulseIn(ECHO1, HIGH);
    distance1 = duration * 0.0343 / 2;
    if (distance1 > 10) distance1 = 0;

    // Đo cảm biến 2
    digitalWrite(TRIG2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG2, LOW);

    duration = pulseIn(ECHO2, HIGH);
    distance2 = duration * 0.0343 / 2;
    if (distance2 > 10) distance2 = 0;
}
  