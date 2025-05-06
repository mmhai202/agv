#include "Vehicle.h"
#include "Uart2.h"
#include "BlynkHandler.h"
#include "Encoder.h"
#include "AStar.h"

// Khai báo các đối tượng cần thiết
Vehicle       v;   // Quản lý xe
Uart2         u;   // Nhận dữ liệu từ raspberry pi camera
BlynkHandler  b;   // Điều khiển xe qua Blynk
Encoder       e;   // Đo vị trí xe
AStar         a;   // Thuật toán A*

void taskRunBlynk(void* pvParameters) {
  while (1) {
    b.run();
    if (v.startMission) {
      if      (abs(u.angle) <= 20)            v.dir = POS_Y;
      else if (abs(u.angle - 90) <= 20)       v.dir = POS_X;
      else if (abs(u.angle + 90) <= 20)       v.dir = NEG_X;
      else if (abs(abs(u.angle) - 180) <= 20) v.dir = NEG_Y;
      v.setMission(0, v.start);
      for (size_t i = 0; i < v.steps.size(); ++i) {Serial.printf("%d,",v.steps[i].id);}
      v.state = START;
      v.startMission = false;
      v.stepIdx = 0;
      v.running = true;
      b.write(9,v.running);
      Serial.println("START");
    }
    if (v.readyRaspi == true && v.readyBlynk == true && v.raspi == false) {
      b.write(7, 1);
      v.raspi = true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void taskForward(void* pvParameters) {
  while (1)
  {
    if (v.fe) {
      if (!v.startEncoder) {
        e.start();
        v.startEncoder = true;
      }
      if (abs(v.eX) > 50) {
        v.left(v.speed);
        v.right(v.speed + 5 * (e.getL() - e.getR() - (v.eX > 0 ? 5 : -3)));
      } else {
        v.left(v.speed);
        v.right(v.speed + 5 * (e.getL() - e.getR()));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskl90(void* pvParameters) {
  while (1)
  {
    if (v.l90) {
      if (!v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 28) {
          v.left(v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.f10 = true;
          v.startEncoder = false;
          Serial.printf("F10:%d,%d ", e.getL(), e.getR());
        }
      }
      else if (v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getL() < 50) {
          v.left(-v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.l90 = false;
          v.f10 = false;
          v.startEncoder = false;
          v.eX = false;
          v.fe = true;
          Serial.printf("L90:%d,%d ", e.getL(), e.getR());
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskr90(void* pvParameters) {
  while (1)
  {
    if (v.r90) {
      if (!v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 28) {
          v.left(v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.f10 = true;
          v.startEncoder = false;
          Serial.printf("F10:%d,%d ", e.getL(), e.getR());
        }
      }
      else if (v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 50) {
          v.left(v.speed);
          v.right(-v.speed - 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.r90 = false;
          v.f10 = false;
          v.startEncoder = false;
          v.eX = false;
          v.fe = true;
          Serial.printf("R90:%d,%d ", e.getL(), e.getR());
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskBack(void* pvParameters) {
  while (1)
  {
    if (v.be) {
      if (!v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 28) {
          v.left(v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.f10 = true;
          v.startEncoder = false;
          Serial.printf("F10:%d,%d ", e.getL(), e.getR());
        }
      }
      else if (v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getL() < 101) {
          v.left(-v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.be = false;
          v.f10 = false;
          v.startEncoder = false;
          v.eX = 0;
          v.fe = true;
          Serial.printf("Back:%d,%d ", e.getL(), e.getR());
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskAlignQR(void* pvParameters) {
  while (1) {
    if (v.alignQR) {
      // PD Control
      float deA = v.eA - v.prev_eA;
      int control = 1.2 * v.eA + 0.8 * deA;
      if (control > 10) control = 10;
      else if (control < -10) control = -10;
    
      // Nếu sai số đáng kể thì điều chỉnh
      if (v.eA < -1.5) {
        v.left(-v.speed - control);
        v.right(v.speed + control);
      } else if (v.eA > 1.5) {
        v.left(v.speed + control);
        v.right(-v.speed - control);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      v.stop();
    
      v.prev_eA = v.eA; // cập nhật eA cũ
      v.alignQR = false;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void taskControl(void* pvParameters) {
  while (1) {
    bool qrAvailable = v.checkQRCode();
    if (qrAvailable && !v.l90 && !v.r90 && !v.be) {
      v.qrData = u.read();
      if (v.qrData.valid) {
        v.readyRaspi = true;
        if (v.steps.size() > 0) {
          float targetAngle;
          // Đặt giá trị targetAngle dựa trên hướng di chuyển của xe
          if      (v.steps[v.stepIdx].dir == POS_X) {targetAngle = 90.0;}
          else if (v.steps[v.stepIdx].dir == NEG_X) {targetAngle = -90.0;}
          else if (v.steps[v.stepIdx].dir == POS_Y) {targetAngle = 0.0;}
          else if (v.steps[v.stepIdx].dir == NEG_Y) {targetAngle = 180.0;}
          // Tính sai số  góc
          if (v.qrData.angle < -170) v.qrData.angle+=360;
          v.eA = v.qrData.angle - targetAngle;
          Serial.printf("eA=%.2f,",v.eA);
        }
        if ((v.state == DONE_STEP || v.state == START) && v.qrData.id == v.steps[v.stepIdx].id) {
          v.fe = false;
          v.startEncoder = false;
          if (e.getL() > 0 || e.getR() > 0) e.stop();
          Serial.printf("Forward:%d,%d ", e.getL(), e.getR());
          v.stop();
          v.state = RUN_QR;
          vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (v.state == RUN_QR) {
          if (abs(v.eA) <= 1.5) {
            v.eX = v.qrData.x-320;
            Serial.printf("eX=%d\n",v.eX);
            v.alignQR = false;
            v.stop();
            v.state = DONE_QR;
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (v.stepIdx+1 < v.steps.size()) {
              v.stepIdx++;
              v.dir = v.steps[v.stepIdx].dir;
              Serial.printf("stepIdx=%d - ",v.stepIdx);
            } else {
              v.state = STOP;
              if (v.qrData.id == v.start) {
                Serial.println("Start arrived.");
                v.setMission(v.start, v.goal);
                for (size_t i = 0; i < v.steps.size(); ++i) {Serial.printf("%d,",v.steps[i].id);}
                v.state = START;
                v.stepIdx = 0;
              } else if (v.qrData.id == v.goal) {
                Serial.println("Mission complete.");
                v.running = false;
                b.write(9,v.running);
              }
            }
          } else v.alignQR = true;
        } 
      } else {
        if (v.state == DONE_STEP && e.getL()>28) {
          v.fe = false;
          v.startEncoder = false;
          if (e.getL() > 0 || e.getR() > 0) e.stop();
          Serial.printf("Forward:%d,%d ", e.getL(), e.getR());
          v.stop();
          v.state = RUN_QR;
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
    }
    if (v.state == DONE_QR) {v.processSteps(); v.state = RUN_STEP;}
    if (!qrAvailable && v.state == RUN_STEP) {v.state = DONE_STEP;}
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  e.begin();
  a.begin();
  u.begin(&Serial2);
  v.begin(&e, &u, &a);
  b.begin(&v);
  b.write(7, 0);

  // Tạo các task FreeRTOS
  xTaskCreate(taskRunBlynk, "TaskRunBlynk", 10000, NULL, 3, NULL);
  xTaskCreate(taskControl, "TaskControl", 10000, NULL, 2, NULL);
  xTaskCreate(taskForward, "taskForward", 10000, NULL, 1, NULL);
  xTaskCreate(taskl90, "taskl90", 10000, NULL, 1, NULL);
  xTaskCreate(taskr90, "taskr90", 10000, NULL, 1, NULL);
  xTaskCreate(taskBack, "taskr90", 10000, NULL, 1, NULL);
  xTaskCreate(taskAlignQR, "taskAlignQR", 10000, NULL, 1, NULL);
}

void loop() {}