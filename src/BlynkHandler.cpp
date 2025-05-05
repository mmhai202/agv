#include "Config.h"
#include <BlynkSimpleEsp32.h>
#include "BlynkHandler.h"

Vehicle*  BlynkHandler::vehicle = nullptr;

void BlynkHandler::begin(Vehicle* v){ 
  printf("Connecting to Blynk...\n");
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  printf("Connected to Blynk!\n");
  BlynkHandler::vehicle = v;
  BlynkHandler::vehicle->readyBlynk = true;
}

void BlynkHandler::run() { Blynk.run();}

void BlynkHandler::write(int pin, int value) { Blynk.virtualWrite(pin, value);}

BLYNK_WRITE(V0) { 
  if(param.asInt()) {
    BlynkHandler::vehicle->left(BlynkHandler::vehicle->speed);
    BlynkHandler::vehicle->right(BlynkHandler::vehicle->speed);
  } else {
    BlynkHandler::vehicle->stop();
  }
}

BLYNK_WRITE(V1) {
  if(param.asInt()) {
    BlynkHandler::vehicle->left(-(BlynkHandler::vehicle->speed));
    BlynkHandler::vehicle->right(-(BlynkHandler::vehicle->speed));
  } else {
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