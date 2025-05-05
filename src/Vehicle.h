#pragma once
#include <Arduino.h>
#include "Encoder.h"
#include "Uart2.h"
#include "AStar.h"

// Các loại hành động
enum Action {IDLE, FWD, BACK, LEFT, RIGHT};
enum Direction {POS_X, NEG_X, POS_Y, NEG_Y};
enum State {START, RUN_STEP, DONE_STEP, RUN_QR , DONE_QR, STOP};

// Cấu trúc bước di chuyển
struct Step {
  int id;
  Action act;
  Direction dir;
};

class Vehicle {
public:
  void begin(Encoder* encoder, Uart2* uart2, AStar* astar);
  void stop();
  void left(int16_t pwm);
  void right(int16_t pwm);
  void setMission(int start, int goal);
  void buildSteps(const std::vector<Node*>& path);
  void startStep(const Step& step);
  void processSteps();
  bool checkQRCode();
  
  int start = 0;
  int goal = 0;
  int speed = 40;
  Direction dir = POS_Y;
  State state = STOP;
  size_t stepIdx = 0;
  std::vector<Step> steps;
  Data qrData;
  int eA;
  int prev_eA = 0;
  bool alignQR = false;
  bool startMission = false;
  bool fe = false;
  bool f10 = false;
  bool l90 = false;
  bool r90 = false;
  bool startEncoder = false;
  bool readyRaspi = false;
  bool raspi = false;
private:
  Encoder* _encoder;
  Uart2* _uart2;
  AStar* _astar;
};