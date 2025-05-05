#include "Config.h"
#include "Vehicle.h"

void Vehicle::begin(Encoder* encoder, Uart2* uart2, AStar* astar) {
  _encoder = encoder;
  _uart2 = uart2;
  _astar = astar;
  pinMode(L_DIR, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT); pinMode(R_PWM, OUTPUT);
  ledcSetup(PWM_CH_LEFT , PWM_FREQ, PWM_RES);  // Setup PWM for left motor
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);  // Setup PWM for right motor
  ledcAttachPin(L_PWM, PWM_CH_LEFT);       // Attach PWM to left motor
  ledcAttachPin(R_PWM, PWM_CH_RIGHT);      // Attach PWM to right motor
  stop();
}
void Vehicle::stop() { 
  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);
  ledcWrite(PWM_CH_LEFT,0);
  ledcWrite(PWM_CH_RIGHT,0);
}

void Vehicle::left(int16_t pwm) {
  if(pwm >= 0) {
    digitalWrite(L_DIR, LOW);
    ledcWrite(PWM_CH_LEFT, pwm);
  } else {
    digitalWrite(L_DIR, HIGH);
    ledcWrite(PWM_CH_LEFT, -pwm);
  }
}

void Vehicle::right(int16_t pwm) { 
  if(pwm >= 0) {
    digitalWrite(R_DIR, LOW); 
    ledcWrite(PWM_CH_RIGHT, pwm);
  } else {
    digitalWrite(R_DIR, HIGH); 
    ledcWrite(PWM_CH_RIGHT, -pwm);
  }
}

//               ↓
//              posY
//        0   1   2   3    
// → posX 4   5   6   7
//        8   9  10  11
void Vehicle::buildSteps(const std::vector<Node*>& path) {
  steps.clear();
  stepIdx = 0;
  steps.push_back({path[0]->id ,Action::IDLE, dir});
  Direction path_dir = dir;

  for (size_t i = 1; i < path.size(); ++i) {
    int dx = path[i]->x - path[i-1]->x;
    int dy = path[i]->y - path[i-1]->y;

    /* ----- hướng POS_Y ----- */
    if (path_dir == POS_Y) {
      if      (dy ==  1) { steps.push_back({path[i]->id, Action::FWD , POS_Y});  path_dir = POS_Y;  }
      else if (dy == -1) { steps.push_back({path[i]->id, Action::BACK, NEG_Y});  path_dir = NEG_Y;  }
      else if (dx ==  1) { steps.push_back({path[i]->id, Action::LEFT, POS_X});  path_dir = POS_X;  }
      else if (dx == -1) { steps.push_back({path[i]->id, Action::RIGHT,NEG_X});  path_dir = NEG_X;  }
      continue;   // *** ĐÃ push -> sang node kế tiếp ***
    }

    /* ----- hướng NEG_Y ----- */
    if (path_dir == NEG_Y) {
      if      (dy ==  1) { steps.push_back({path[i]->id, Action::BACK, NEG_Y});  path_dir = NEG_Y;  }
      else if (dy == -1) { steps.push_back({path[i]->id, Action::FWD , POS_Y});  path_dir = POS_Y;  }
      else if (dx ==  1) { steps.push_back({path[i]->id, Action::RIGHT,POS_X});  path_dir = POS_X;  }
      else if (dx == -1) { steps.push_back({path[i]->id, Action::LEFT ,NEG_X});  path_dir = NEG_X;  }
      continue;
    }

    /* ----- hướng POS_X ----- */
    if (path_dir == POS_X) {
      if      (dy ==  1) { steps.push_back({path[i]->id, Action::RIGHT,POS_Y});  path_dir = POS_Y;  }
      else if (dy == -1) { steps.push_back({path[i]->id, Action::LEFT ,NEG_Y});  path_dir = NEG_Y;  }
      else if (dx ==  1) { steps.push_back({path[i]->id, Action::FWD  ,POS_X});  path_dir = POS_X;  }
      else if (dx == -1) { steps.push_back({path[i]->id, Action::BACK ,NEG_X});  path_dir = NEG_X;  }
      continue;
    }

    /* ----- hướng NEG_X ----- */
    if (path_dir == NEG_X) {
      if      (dy ==  1) { steps.push_back({path[i]->id, Action::LEFT ,POS_Y});  path_dir = POS_Y;  }
      else if (dy == -1) { steps.push_back({path[i]->id, Action::RIGHT,NEG_Y});  path_dir = NEG_Y;  }
      else if (dx ==  1) { steps.push_back({path[i]->id, Action::BACK ,POS_X});  path_dir = POS_X;  }
      else if (dx == -1) { steps.push_back({path[i]->id, Action::FWD  ,NEG_X});  path_dir = NEG_X;  }
      continue;
    }
  }
}

void Vehicle::startStep(const Step& step) {
  switch (step.act) {
    case Action::FWD:
      fe = true;
      break;
    case Action::BACK:
      left(-speed);
      right(-speed);
      break;
    case Action::LEFT:
      l90 = true;
      break;
    case Action::RIGHT:
      r90 = true;
      break;
    default:
      break;
  }
}

void Vehicle::processSteps() {
  startStep(steps[stepIdx]);
}

void Vehicle::setMission(int start, int goal) {
  _astar->reset();
  _astar->setMission(start, goal);
  std::vector<Node*> path = _astar->findPath(_astar->start, _astar->goal);
  if (path.empty()) {
    Serial.println("[ERROR] Path not found!");
    return;
  }
  buildSteps(path);
}


bool Vehicle::checkQRCode() {
  return _uart2->available();
}