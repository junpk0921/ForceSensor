
/* ******************************************
** EZMAKER Online (EZ-ON) code
** ******************************************/
/* <include-default> */
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "ezmaker_v2.1.h"

#define getAnalogData analogRead

/* <include-modules> */

/* <lib-default> */

/* <lib-modules> */
const uint8_t SLAVE_ADDR = 0x08;
#define CALIB_BUTTON EZ_D0

// STM32와 약속한 명령 코드
#define CMD_SAVE_CALIB_RATIO  0xC1

typedef union {
  struct __attribute__((packed)) {
    int32_t weight_g100;     
    int16_t angle_deg100;     
    int16_t accX_g1000;      
    int16_t accY_g1000;       
    int16_t accZ_g1000;     
  } val;
  uint8_t buffer[12];
} Packet_t;


/* <variable-modules> */
const float TARGET_WEIGHT_G = 100.0f;
const float DEADBAND_WEIGHT = 0.5f;

float correction_factor = 1.0f; 
float offset_weight = 0.0f;

bool slave_online = false; 
bool need_calibration = false; 

// 스위치 디바운싱 및 상태 관리
bool lastSwitchState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 150;

/* <declare-modules> */
// [ 데이터 패킷 읽기 ]
bool readPacket(Packet_t &p) {
  if (Wire.requestFrom(SLAVE_ADDR, (uint8_t)12) == 12) {
    for (int i = 0; i < 12; i++) {
      p.buffer[i] = Wire.read();
    }
    return true;
  }
  return false;
}

// [ STM32에 보정 비율 전송 ]
bool sendCalibrationRatioToSTM32(float ratio) {
  if (ratio < 0.1f || ratio > 10.0f) {
    return false;
  }

  uint8_t ratioBytes[4];
  memcpy(ratioBytes, &ratio, sizeof(float));

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(CMD_SAVE_CALIB_RATIO);
  Wire.write(ratioBytes, 4);

  uint8_t result = Wire.endTransmission();
  return result == 0;
}

// [ 초기 영점 보정 ]
void calibrateSensors() {
  float sum_w = 0.0f;
  const int samples = 50; 
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    Packet_t p;
    if (readPacket(p)) {
      sum_w += (p.val.weight_g100 / 100.0f);
      validSamples++;
    }
    delay(5);
  }

  if (validSamples > 0) {
    offset_weight = sum_w / validSamples;
  }
}

// [ 캘리브레이션 스위치 입력 체크 ]
void checkCalibrationSwitch() {
  bool currentSwitchState = digitalRead(CALIB_BUTTON);
  unsigned long currentTime = millis();

  if (currentSwitchState == HIGH && lastSwitchState == LOW) {
    if (currentTime - lastDebounceTime > DEBOUNCE_DELAY) {
      need_calibration = true;
      lastDebounceTime = currentTime;
    }
  }
  lastSwitchState = currentSwitchState;
}


/* <code-setup> */
void setup() {
  Serial.begin(115200);
  pinMode(CALIB_BUTTON, INPUT);
  Wire.begin();

  delay(3000);

  calibrateSensors();
  slave_online = true;
}

/* <code-loop> */
void loop() {
  Packet_t data;

  // 스위치 감시
  checkCalibrationSwitch();

  if (readPacket(data)) {
    if (!slave_online) {
      delay(500);
      calibrateSensors(); 
      slave_online = true;
    }

    float raw_measured_g = (data.val.weight_g100 / 100.0f - offset_weight);

    // [ D0 스위치 기반 STM32 영구 보정 ]
    if (need_calibration && abs(raw_measured_g) > 10.0f) { 
      delay(1500); 

      Packet_t stable_data;
      if (readPacket(stable_data)) {
        float stable_weight = (stable_data.val.weight_g100 / 100.0f - offset_weight);

        if (abs(stable_weight) > 1.0f) {
          float ratio = TARGET_WEIGHT_G / stable_weight;
          bool save_ok = sendCalibrationRatioToSTM32(ratio);

          if (save_ok) {
            correction_factor = 1.0f; // 보정이 STM32측에 반영되므로 마스터 배율 초기화
            Serial.print("CALIB_SAVED_TO_STM32:");
            Serial.println(ratio, 6);
            delay(500); // 적용 대기 시간
          } else {
            Serial.println("CALIB_SAVE_FAILED");
          }
        }
        need_calibration = false;
      }
    }

    // 최종 무게 계산 및 출력
    float weight_g = raw_measured_g * correction_factor;

    if (abs(weight_g) < DEADBAND_WEIGHT) {
      weight_g = 0.0f;
    }
   
    Serial.print("Weight_g:");
    Serial.println(weight_g, 1);          
  } 
  else {
    slave_online = false;
  }

  delay(20);
}