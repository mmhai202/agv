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
      v.setMission(v.start, v.goal);
      for (size_t i = 0; i < v.steps.size(); ++i) {
        Serial.printf("[%02d] id=%d act=%d dir=%d\n",i,v.steps[i].id,v.steps[i].act,v.steps[i].dir);
      }
      v.state = START;
      v.startMission = false;
      Serial.println("START");
    }
    if (v.readyRaspi == true) {
      b.write(7, 1);
      v.readyRaspi = false;
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
      v.left(v.speed);
      v.right(v.speed + 5 * (e.getL() - e.getR()));
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
          Serial.printf("EncoderL: %d | EncoderR: %d\n", e.getL(), e.getR());
        }
      }
      else if (v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getL() < 53) {
          v.left(-v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.l90 = false;
          v.f10 = false;
          v.startEncoder = false;
          v.fe = true;
          Serial.printf("EncoderL: %d | EncoderR: %d\n", e.getL(), e.getR());
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
          Serial.printf("EncoderL: %d | EncoderR: %d\n", e.getL(), e.getR());
        }
      }
      else if (v.f10) {
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 53) {
          v.left(v.speed);
          v.right(-v.speed - 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.r90 = false;
          v.f10 = false;
          v.startEncoder = false;
          v.fe = true;
          Serial.printf("EncoderL: %d | EncoderR: %d\n", e.getL(), e.getR());
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
      int deA = v.eA - v.prev_eA;
      int control = 1.2 * v.eA + 0.8 * deA;
      if (control > 10) control = 10;
      else if (control < -10) control = -10;
    
      // Nếu sai số đáng kể thì điều chỉnh
      if (v.eA > 3) {
        v.left(-v.speed - control);
        v.right(v.speed + control);
      } else if (v.eA < -3) {
        v.left(v.speed + control);
        v.right(-v.speed - control);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      v.stop();
    
      v.prev_eA = v.eA; // cập nhật eA cũ
      v.alignQR = false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void taskControl(void* pvParameters) {
  while (1) {
    bool qrAvailable = v.checkQRCode();
    if (qrAvailable) {
      v.qrData = u.read();
      Serial.println("read data");

      if (v.qrData.valid) {
        Serial.println("data valid");
        v.readyRaspi = true;
        if (v.steps.size() > 0) {
          int targetAngle;
          // Đặt giá trị targetAngle dựa trên hướng di chuyển của xe
          if      (v.steps[v.stepIdx].dir == POS_X) {targetAngle = 270;}
          else if (v.steps[v.stepIdx].dir == NEG_X) {targetAngle = 90;}
          else if (v.steps[v.stepIdx].dir == POS_Y) {targetAngle = 0;}
          else if (v.steps[v.stepIdx].dir == NEG_Y) {targetAngle = 180;}
          // Tính sai số  góc
          if(v.qrData.angle >= 180 && v.steps[v.stepIdx].dir == POS_Y) {v.qrData.angle -= 360;}
          v.eA = v.qrData.angle - targetAngle;
          Serial.printf("v.eA = %d\n", v.eA);
        }
      }

      if ((v.state == DONE_STEP || v.state == START) && v.qrData.id == v.steps[v.stepIdx].id) {
        Serial.printf("RUN_QR\n");
        v.fe = false;
        v.startEncoder = false;
        e.stop();
        v.stop();
        v.state = RUN_QR;
        vTaskDelay(pdMS_TO_TICKS(1000));
      } else if (v.state == RUN_QR) {
        if (abs(v.eA) <= 3) {
          v.alignQR = false;
          v.stop();
          v.state = DONE_QR;
          if (v.stepIdx+1 < v.steps.size()) {
            v.dir = v.steps[v.stepIdx].dir;
            v.stepIdx++;
            Serial.print("stepIdx: "); Serial.println(v.stepIdx);
          } else {
            v.state = STOP;
            v.stepIdx = 0;
            v.startMission = false;
            Serial.println("Mission complete.");
          }
        } else v.alignQR = true;
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
  Serial.println("setup.");

  // Tạo các task FreeRTOS
  xTaskCreate(taskRunBlynk, "TaskRunBlynk", 10000, NULL, 3, NULL);
  xTaskCreate(taskControl, "TaskControl", 10000, NULL, 2, NULL);
  xTaskCreate(taskForward, "taskForward", 10000, NULL, 1, NULL);
  xTaskCreate(taskl90, "taskl90", 10000, NULL, 1, NULL);
  xTaskCreate(taskr90, "taskr90", 10000, NULL, 1, NULL);
  xTaskCreate(taskAlignQR, "taskAlignQR", 10000, NULL, 1, NULL);
}

void loop() {}
