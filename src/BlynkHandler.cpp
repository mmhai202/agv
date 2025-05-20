#include "Config.h"
#include <BlynkSimpleEsp32.h>
#include "BlynkHandler.h"

Vehicle*  BlynkHandler::vehicle = nullptr;
Encoder*  BlynkHandler::encoder = nullptr;

void BlynkHandler::begin(Vehicle* v, Encoder* e){ 
  printf("Connecting to Blynk...\n");
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  printf("Connected to Blynk!\n");
  BlynkHandler::vehicle = v;
  BlynkHandler::encoder = e;
  BlynkHandler::vehicle->readyBlynk = true;
}

void BlynkHandler::run() { Blynk.run();}

void BlynkHandler::write(int pin, int value) { Blynk.virtualWrite(pin, value);}

BLYNK_WRITE(V0) { // Khi bấm phím tiến
  if(param.asInt()) {
    BlynkHandler::vehicle->fe = true;
    BlynkHandler::vehicle->estimate_v = true;
  } else {
    BlynkHandler::vehicle->fe = false;
    BlynkHandler::vehicle->estimate_v = false;
    BlynkHandler::encoder->reset();
    BlynkHandler::vehicle->stop();
  }
}

BLYNK_WRITE(V1) { // Khi bấm phím lùi
  if(param.asInt()) {
    BlynkHandler::vehicle->be = true;
    BlynkHandler::vehicle->estimate_v = true;
  } else {
    BlynkHandler::vehicle->be = false;
    BlynkHandler::vehicle->estimate_v = false;
    BlynkHandler::encoder->reset();
    BlynkHandler::vehicle->stop();
  }
}

BLYNK_WRITE(V2) { 
  if(param.asInt()) {
    BlynkHandler::vehicle->left(-BlynkHandler::vehicle->speed);
    BlynkHandler::vehicle->right(BlynkHandler::vehicle->speed);
  } else {
    BlynkHandler::vehicle->stop();
  }
}

BLYNK_WRITE(V3) { 
  if(param.asInt()) {
    BlynkHandler::vehicle->left(BlynkHandler::vehicle->speed);
    BlynkHandler::vehicle->right(-BlynkHandler::vehicle->speed);
  } else {
    BlynkHandler::vehicle->stop();
  }
}

BLYNK_WRITE(V4) { BlynkHandler::vehicle->speed = map(param.asInt(),1,10,1,10)*10; 
                 printf("Speed: %d\n", BlynkHandler::vehicle->speed);}

BLYNK_WRITE(V5) { 
  BlynkHandler::vehicle->start = param.asInt();
}

BLYNK_WRITE(V6) { 
  BlynkHandler::vehicle->goal = param.asInt();
}

BLYNK_WRITE(V8) {
  if(param.asInt()) {
    BlynkHandler::vehicle->startMission = true;
  }
}

BLYNK_WRITE(V12) {
  if(param.asInt()) {
    BlynkHandler::vehicle->nang();
  }
}

BLYNK_WRITE(V13) {
  if(param.asInt()) {
    BlynkHandler::vehicle->ha();
  }
}

BLYNK_WRITE(V14) {
  if(param.asInt()) {
    BlynkHandler::vehicle->fe = false;
    BlynkHandler::vehicle->be = false;
    BlynkHandler::vehicle->r90 = false;
    BlynkHandler::vehicle->l90 = false;
    BlynkHandler::vehicle->l180 = false;
    BlynkHandler::vehicle->running = false;
    Blynk.virtualWrite(V9, 0);
    BlynkHandler::vehicle->stop();
    BlynkHandler::vehicle->state = STOP;
  }
}