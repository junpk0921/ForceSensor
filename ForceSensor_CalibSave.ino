
/* ******************************************
** EZMAKER Online (EZ-ON) code
** ******************************************/
/* <include-default> */
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <string.h>
#include "ezmaker_v2.1.h"

#define getAnalogData analogRead

/* <include-modules> */

/* <lib-default> */

/* <lib-modules> */
const uint8_t SLAVE_ADDR = 0x08;
#define CALIB_BUTTON EZ_D0

// STM32와 약속한 명령 코드 (영구 저장용)
#define CMD_SAVE_CALIB_RATIO  0xC1

// STM32 통신용 14바이트 패킷 구조체
typedef union {
  struct __attribute__((packed)) {
    int32_t weight_g100;      // g x 100
    int32_t force_N1000;      // N x 1000
    int16_t accX_g1000;       // g x 1000
    int16_t accY_g1000;       // g x 1000
    int16_t accZ_g1000;       // g x 1000
  } val;
  uint8_t buffer[14];
} Packet_t;


/* <variable-modules> */
const float TARGET_WEIGHT_G = 100.0f;

bool slave_online = false;
bool need_calibration = false;

// I2C 안정화용 변수
uint8_t i2c_fail_count = 0;
const uint8_t I2C_FAIL_LIMIT = 10;

// 스위치 제어 변수
bool lastSwitchState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 150;

/* <declare-modules> */
// [ I2C 버스 물리 복구 ] SDA Stuck 현상 해제
void recoverI2CBus() {
  pinMode(EZ_SDA, INPUT_PULLUP);
  pinMode(EZ_SCL, INPUT_PULLUP);
  delay(5);

  pinMode(EZ_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(EZ_SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite(EZ_SCL, LOW);
    delayMicroseconds(10);
  }
  digitalWrite(EZ_SCL, HIGH);
  delayMicroseconds(10);

  pinMode(EZ_SDA, INPUT_PULLUP);
  pinMode(EZ_SCL, INPUT_PULLUP);
  delay(5);
}

// [ I2C 마스터 재초기화 ]
void resetI2CMaster() {
  Wire.end();
  delay(50);
  recoverI2CBus();
  Wire.begin();
  delay(100);
}

// [ 데이터 패킷 읽기 ]
bool readPacket(Packet_t &p) {
  uint8_t received = Wire.requestFrom(SLAVE_ADDR, (uint8_t)14);
  if (received == 14) {
    for (int i = 0; i < 14; i++) p.buffer[i] = Wire.read();
    return true;
  }
  while (Wire.available()) { Wire.read(); }
  return false;
}

// [ 슬레이브 준비 대기 ]
bool waitForSlaveReady() {
  Packet_t dummy;
  for (int i = 0; i < 50; i++) {
    if (readPacket(dummy)) return true;
    delay(100);
  }
  return false;
}

// [ STM32 보정 데이터 전송 ]
bool sendCalibrationRatioToSTM32(float ratio) {
  if (ratio < 0.1f || ratio > 10.0f) return false;

  uint8_t ratioBytes[4];
  memcpy(ratioBytes, &ratio, sizeof(float));

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(CMD_SAVE_CALIB_RATIO);
  Wire.write(ratioBytes, 4);
  return (Wire.endTransmission() == 0);
}

// [ 보정용 평균 무게 계산 ]
bool readStableWeight(float &avgWeight_g) {
  const int samples = 30;
  int validSamples = 0;
  float sumWeight = 0.0f;

  for (int i = 0; i < samples; i++) {
    Packet_t p;
    if (readPacket(p)) {
      sumWeight += (p.val.weight_g100 / 100.0f);
      validSamples++;
    } else {
      i2c_fail_count++;
      if (i2c_fail_count >= I2C_FAIL_LIMIT) { resetI2CMaster(); i2c_fail_count = 0; }
    }
    delay(10);
  }
  if (validSamples > 0) { avgWeight_g = sumWeight / validSamples; return true; }
  return false;
}

// [ 캘리브레이션 버튼 체크 ]
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
  delay(500);

  recoverI2CBus();
  Wire.begin();
  delay(3000);

  slave_online = waitForSlaveReady();
  i2c_fail_count = 0;
}

/* <code-loop> */
void loop() {
  Packet_t data;
  checkCalibrationSwitch();

  if (readPacket(data)) {
    slave_online = true;
    i2c_fail_count = 0;
    float weight_g = data.val.weight_g100 / 100.0f;

    // [ 보정 트리거 처리 ]
    if (need_calibration) {
      delay(1500); 
      float stable_weight_g = 0.0f;

      if (readStableWeight(stable_weight_g)) {
        if (abs(stable_weight_g) > 1.0f) {
          float ratio = TARGET_WEIGHT_G / stable_weight_g;
          if (sendCalibrationRatioToSTM32(ratio)) {
            Serial.print("CALIB_SAVED_TO_STM32:");
            Serial.println(ratio, 6);
            delay(500);
          } else {
            Serial.println("CALIB_SAVE_FAILED");
            resetI2CMaster();
          }
        } else {
          Serial.println("CALIB_WEIGHT_TOO_SMALL");
        }
      } else {
        Serial.println("CALIB_READ_FAILED");
        resetI2CMaster();
      }
      need_calibration = false;
    }

    // 데이터 출력
    Serial.print("Weight_g:");
    Serial.println(weight_g, 1);
  }
  else {
    slave_online = false;
    i2c_fail_count++;
    if (i2c_fail_count >= I2C_FAIL_LIMIT) {
      resetI2CMaster();
      i2c_fail_count = 0;
    }
  }
  delay(20);
}