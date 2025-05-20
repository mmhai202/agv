#pragma once

// BlynkHandler
#define BLYNK_TEMPLATE_ID   "TMPL6i0G0w7eM"
#define BLYNK_TEMPLATE_NAME "agv"
#define BLYNK_AUTH_TOKEN    "iqpd9uR8FqKjTXxOW5QeLO_43JlZymip"

// WiFi
constexpr char WIFI_SSID[] = "ABC"; // constexpr giúp xác định giá trị và hàm tại thời điểm biên dịch
constexpr char WIFI_PASS[] = "03072002";

// Vehicle
constexpr int L_DIR = 19;
constexpr int L_PWM = 21;
constexpr int R_DIR = 5;
constexpr int R_PWM = 18;
constexpr int PWM_CH_LEFT  = 0;
constexpr int PWM_CH_RIGHT = 1;
constexpr int PWM_FREQ     = 5000;
constexpr int PWM_RES      = 8;

// Encoder
constexpr int ENCODER_L = 27;
constexpr int ENCODER_R = 14;

// HC-SR04
constexpr int TRIG1 = 33;
constexpr int ECHO1 = 32;
constexpr int TRIG2 = 26;
constexpr int ECHO2 = 25;

// Động cơ nâng
constexpr int LIFT_DIR = 22;
constexpr int LIFT_PWM = 23;
constexpr int PWM_CH_LIFT = 2;

// Chân đọc pin
constexpr int BATTERY_PIN = 13;

// UART2 data
struct Data {
  bool valid;
  int id;
  int x;
  int y;
  float angle;
};