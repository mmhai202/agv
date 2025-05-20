#include "Vehicle.h"
#include "Uart2.h"
#include "BlynkHandler.h"
#include "Encoder.h"
#include "AStar.h"
#include "Hcsr04.h"

// Khai báo các đối tượng cần thiết
Vehicle       v;   // Quản lý xe
Uart2         u;   // Nhận dữ liệu từ raspberry pi camera
BlynkHandler  b;   // Điều khiển xe qua Blynk
Encoder       e;   // Đo vị trí xe
AStar         a;   // Thuật toán A*
hcsr04        h;   // Cảm biến siêu âm

const int QUAY_TRAI = 54;
const int QUAY_PHAI = 54;
const int QUAY_180  = 105;

unsigned long pre_time, new_time, first_time = 0;
float prev_v = 0;
// Task đọc vận tốc và gia tốc
void taskDoVanToc(void* pvParameters) {
  while(1)
  {
    if (v.estimate_v && e.getL() > 0) {
      float d = (float)e.getL() / 80.0 * 314.0; // quãng đường mm
      new_time = millis();

      // Vận tốc = quãng đường / thời gian từ đầu
      unsigned long total_time = new_time - first_time;
      float curr_v = 0;
      if (total_time > 0) {
        curr_v = d / ((float)total_time / 1000.0); // mm/s
      }

      // Gia tốc = (v2 - v1) / delta_t
      unsigned long delta_time = new_time - pre_time;
      float acc = 0;
      if (delta_time > 0) {
        acc = (curr_v - prev_v) / ((float)delta_time / 1000.0); // mm/s^2
      }

      // In kết quả
      Serial.printf("van toc: %.2f mm/s\tgia toc: %.2f mm/s^2\n", curr_v, acc);

      // Cập nhật lại giá trị trước đó
      prev_v = curr_v;
      pre_time = new_time;

    } else if (v.estimate_v && e.getL() == 0) {
      e.start(); // reset encoder
      first_time = millis();
      pre_time = first_time;
      prev_v = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task đọc cảm biến siêu âm HCSR04 để phát hiện vật cản
void taskHCSR04(void* pvParameters) {
  while (1) {
    h.run(); // Cập nhật dữ liệu từ cảm biến siêu âm

    // Nếu phát hiện vật cản phía trước
    if (h.distance1 > 0 && h.distance1 <= 10 && v.state == DONE_STEP) {
      v.osbtacle = true; // Bật cờ vật cản
      b.write(10,v.osbtacle);
      v.fe = false;
      v.startEncoder = false;
      if (e.getL() > 0 || e.getR() > 0) e.stop();
      v.stop();
      v.dir = v.steps[v.stepIdx].dir; // Lưu lại hướng hiện tại
      a.setBlocked(v.steps[v.stepIdx].id); // Đánh dấu node hiện tại bị chặn trong A*

      // Lập kế hoạch lại từ bước trước đến start hoặc goal tùy trạng thái hiện tại
      if (!v.arrivedStart) v.setMission(v.steps[v.stepIdx-1].id, v.start);
      else v.setMission(v.steps[v.stepIdx-1].id, v.goal);
      v.state = START;
      v.stepIdx = 0;
      vTaskDelay(pdMS_TO_TICKS(500));
      v.be = true;
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Nếu có vật cản phía sau khi đang lùi thì dừng lại và báo lỗi
    if (h.distance2 > 0 && h.distance2 <= 10 && v.osbtacle == true) {
      v.be = false;
      v.startEncoder = false;
      if (e.getL() > 0 || e.getR() > 0) e.stop();
      v.stop();
      v.error = true;
      v.state = STOP;
      b.write(11,v.error);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task nhận điều khiển từ app Blynk
void taskRunBlynk(void* pvParameters) {
  while (1) {
    b.run(); // Duy trì tín hiệu từ Blynk

    // Nếu được yêu cầu bắt đầu nhiệm vụ (startMission)
    if (v.startMission) {
      if      (abs(u.angle) <= 20)            v.dir = POS_Y;
      else if (abs(u.angle - 90) <= 20)       v.dir = POS_X;
      else if (abs(u.angle + 90) <= 20)       v.dir = NEG_X;
      else if (abs(abs(u.angle) - 180) <= 20) v.dir = NEG_Y;
      v.setMission(u.id, v.start);
      Serial.printf("Next mission: %d steps\n", v.steps.size());

      // Cập nhật các cờ
      v.state = START;
      v.startMission = false;
      v.stepIdx = 0;
      v.running = true;
      v.error = false;
      v.osbtacle = false;
      b.write(9,v.running);
      b.write(10,v.osbtacle);
      b.write(11,v.error);
      a.clearAllBlocked();
      v.arrivedStart = false;
      Serial.println("START");
    }
    if (v.readyRaspi == true && v.readyBlynk == true && v.raspi == false) {
      b.write(7, 1);
      v.raspi = true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task điều khiển xe tiến về phía trước bằng encoder
void taskForward(void* pvParameters) {
  while (1)
  {
    if (v.fe) { // Nếu cờ "forward encoder" được bật
      if (!v.startEncoder) {
        e.start(); // Bắt đầu đo encoder
        v.startEncoder = true;
      }
      // Điều chỉnh tốc độ để giữ cân bằng dựa vào độ lệch trục X
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

// Task quay đầu 180 độ
void taskl180(void* pvParameters) {
  while (1)
  {
    if (v.l180) {
      if (!v.f10) { // Giai đoạn đầu: đi thẳng 1 đoạn nhỏ 10cm (tương ứng 28 xung encoder)
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 28) {
          v.left(v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()) - 2);
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.f10 = true;
          v.startEncoder = false;
          Serial.printf("F10:%d,%d ", e.getL(), e.getR());
        }
      }
      else if (v.f10) { // Giai đoạn quay đầu 180 độ
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getL() < QUAY_180) {
          v.left(-v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.l180 = false;
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

// Task quay trái 90 độ
void taskl90(void* pvParameters) {
  while (1)
  {
    if (v.l90) {
      if (!v.f10) { // Giai đoạn đầu: đi thẳng 1 đoạn nhỏ 10cm (tương ứng 28 xung encoder)
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 28) {
          v.left(v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()) - 2);
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.f10 = true;
          v.startEncoder = false;
          Serial.printf("F10:%d,%d ", e.getL(), e.getR());
        }
      }
      else if (v.f10) { // Giai đoạn rẽ trái 90 độ
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getL() < QUAY_TRAI) {
          v.left(-v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.l90 = false;
          v.f10 = false;
          v.startEncoder = false;
          v.eX = 0;
          v.fe = true;
          Serial.printf("L90:%d,%d ", e.getL(), e.getR());
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Task quay phải 90 độ
void taskr90(void* pvParameters) {
  while (1)
  {
    if (v.r90) {
      if (!v.f10) { // Giai đoạn đầu: đi thẳng 1 đoạn nhỏ 10cm (tương ứng 28 xung encoder)
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < 28) {
          v.left(v.speed);
          v.right(v.speed + 5 * (e.getL() - e.getR()) - 2);
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.f10 = true;
          v.startEncoder = false;
          Serial.printf("F10:%d,%d ", e.getL(), e.getR());
        }
      }
      else if (v.f10) { // Giai đoạn quay phải 90 độ
        if (!v.startEncoder) {
          e.start();
          v.startEncoder = true;
        }
        if (e.getR() < QUAY_PHAI) {
          v.left(v.speed);
          v.right(-v.speed - 5 * (e.getL() - e.getR()));
        } else {
          v.stop();
          e.stop();
          vTaskDelay(pdMS_TO_TICKS(1000));
          v.r90 = false;
          v.f10 = false;
          v.startEncoder = false;
          v.eX = 0;
          v.fe = true;
          Serial.printf("R90:%d,%d ", e.getL(), e.getR());
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Task điều khiển đi lùi bằng encoder
void taskBack(void* pvParameters) {
  while (1)
  {
    if (v.be) {
      if (!v.startEncoder) {
        e.start();
        v.startEncoder = true;
      }
      v.left(-v.speed);
      v.right(-v.speed - 5 * (e.getL() - e.getR()));
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Task căn chỉnh góc để thẳng hàng với mã QR
void taskAlignQR(void* pvParameters) {
  while (1) {
    if (v.alignQR) {
      // PD Control
      float deA = v.eA - v.prev_eA;
      int control = 1.2 * v.eA + 0.8 * deA;
      if (control > 10) control = 10;
      else if (control < -10) control = -10;
    
      // Nếu sai số đáng kể thì điều chỉnh
      int speed = 40;
      if (v.eA < -1.5) {
        v.left(-speed - control);
        v.right(speed + control);
      } else if (v.eA > 1.5) {
        v.left(speed + control);
        v.right(-speed - control);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      v.stop();
    
      v.prev_eA = v.eA; // cập nhật eA cũ
      v.alignQR = false;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task quản lý luồng điều khiển chính
void taskControl(void* pvParameters) {
  while (1) {
    bool qrAvailable = v.checkQRCode(); 
    if (qrAvailable && !v.l90 && !v.r90 && !v.l180) { // Kiểm tra có mã QR không
      v.qrData = u.read(); // Đọc dữ liệu từ raspi camera qua uart
      if (v.qrData.valid) {
        v.readyRaspi = true;
        if (v.steps.size() > 0 && v.state != DONE_QR && v.state != RUN_STEP) {
          float targetAngle;
          // Đặt giá trị targetAngle dựa trên hướng di chuyển của xe
          if      (v.steps[v.stepIdx].dir == POS_X) {targetAngle = 90.0;}
          else if (v.steps[v.stepIdx].dir == NEG_X) {targetAngle = -90.0;}
          else if (v.steps[v.stepIdx].dir == POS_Y) {targetAngle = 0.0;}
          else if (v.steps[v.stepIdx].dir == NEG_Y) {targetAngle = 180.0;}
          // Tính sai số góc
          if (v.qrData.angle < -160) {
            v.eA = v.qrData.angle + 360 - targetAngle;
          } else v.eA = v.qrData.angle - targetAngle;
        }
        if ((v.state == DONE_STEP || v.state == START) && v.qrData.id == v.steps[v.stepIdx].id) {
          v.fe = false;
          v.be = false;
          v.osbtacle = false;
          v.startEncoder = false;
          if (e.getL() > 0 || e.getR() > 0) e.stop();
          Serial.printf("Forward:%d,%d | ", e.getL(), e.getR());
          v.stop();
          v.state = RUN_QR;
          vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (v.state == RUN_QR) {
          if (abs(v.eA) <= 1.5) {
            v.eX = v.qrData.x-320;
            Serial.printf("eA=%.2f,",v.eA);
            Serial.printf("eX=%d\n",v.eX);
            v.alignQR = false;
            v.stop();
            v.state = DONE_QR;
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (v.stepIdx+1 < v.steps.size()) {
              v.stepIdx++;
              v.dir = v.steps[v.stepIdx].dir;
              Serial.printf("ID=%d - ",v.steps[v.stepIdx].id);
            } else {
              v.state = STOP;
              if (v.qrData.id == v.start) {
                Serial.println("Start arrived.");
                v.nang();
                vTaskDelay(pdMS_TO_TICKS(10000)); // Thời gian chờ nâng hàng
                v.arrivedStart = true;
                v.setMission(v.start, v.goal);
                Serial.printf("Next mission: %d steps\n", v.steps.size());
                v.state = START;
                v.stepIdx = 0;
              } else if (v.qrData.id == v.goal) {
                Serial.println("Goal arrived.");
                Serial.println("Mission complete.");
                v.ha();
                vTaskDelay(pdMS_TO_TICKS(5000)); // Thời gian chờ hạ hàng
                v.running = false;
                b.write(9,v.running);
                v.arrivedStart = false;
                a.clearAllBlocked();
              }
            }
          } else v.alignQR = true;
        } 
      } else {
        if (v.state == DONE_STEP && e.getL()>28) {
          v.fe = false;
          v.be = false;
          v.osbtacle = false;
          v.startEncoder = false;
          if (e.getL() > 0 || e.getR() > 0) e.stop();
          Serial.printf("Forward:%d,%d | ", e.getL(), e.getR());
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

// Hàm setup ban đầu
void setup() {
  Serial.begin(115200);
  e.begin();
  a.begin();
  u.begin(&Serial2);
  v.begin(&e, &u, &a);
  b.begin(&v, &e);
  h.begin();

  // Reset trạng thái hiển thị trên Blynk
  b.write(7, 0); b.write(9, 0); b.write(10, 0); b.write(11, 0);

  // Tạo các task FreeRTOS
  xTaskCreate(taskRunBlynk, "TaskRunBlynk", 10000, NULL, 3, NULL);
  xTaskCreate(taskControl, "TaskControl", 10000, NULL, 2, NULL);
  xTaskCreate(taskForward, "taskForward", 10000, NULL, 1, NULL);
  xTaskCreate(taskl90, "taskl90", 10000, NULL, 1, NULL);
  xTaskCreate(taskl180, "taskl180", 10000, NULL, 1, NULL);
  xTaskCreate(taskr90, "taskr90", 10000, NULL, 1, NULL);
  xTaskCreate(taskBack, "taskBack", 10000, NULL, 1, NULL);
  xTaskCreate(taskAlignQR, "taskAlignQR", 10000, NULL, 1, NULL);
  xTaskCreate(taskHCSR04, "taskHCSR04", 10000, NULL, 2, NULL);
  xTaskCreate(taskDoVanToc, "taskDoVanToc", 10000, NULL, 2, NULL);
}

void loop() {} // Không cần làm gì trong loop vì đã sử dụng freeRTOS