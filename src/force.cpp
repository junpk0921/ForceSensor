
/* ******************************************
** EZMAKER Online (EZ-ON) code
** ******************************************/
/* <include-default> */
#include <Arduino.h>
#include <Wire.h>
#include "ezmaker_v2.1.h"

/* <include-modules> */

/* <lib-default> */

/* <lib-modules> */
const uint8_t SLAVE_ADDR = 0x08;

typedef union {
  struct __attribute__((packed)) {
    int32_t weight_g100;    
    int32_t force_N1000;    
    int16_t accX_g1000;     
    int16_t accY_g1000;     
    int16_t accZ_g1000;     
  } val;
  uint8_t buffer[14];
} Packet_t;


/* <variable-modules> */
uint8_t fail_count = 0;
const int8_t FORCE_DIRECTION = 1;

/* <declare-modules> */
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

void resetI2CMaster() {
  Wire.end();
  delay(50);
  recoverI2CBus();
  Wire.begin();
  delay(100);
}

bool readPacket(Packet_t &p) {
  uint8_t received = Wire.requestFrom(SLAVE_ADDR, (uint8_t)14);

  if (received == 14) {
    for (int i = 0; i < 14; i++) {
      p.buffer[i] = Wire.read();
    }
    return true;
  }

  while (Wire.available()) {
    Wire.read();
  }
  return false;
}


/* <code-setup> */
void setup() {
  Serial.begin(115200);
  delay(500);

  recoverI2CBus();
  Wire.begin();
  delay(3000);

  fail_count = 0;
}

/* <code-loop> */
void loop() {
  Packet_t data;

  if (readPacket(data)) {
    fail_count = 0;

    float force_N   = (data.val.force_N1000 / 1000.0f) * FORCE_DIRECTION;
    float weight_kg = data.val.weight_g100 / 100000.0f;

    float ax = data.val.accX_g1000 / 1000.0f;
    float ay = data.val.accY_g1000 / 1000.0f;
    float az = data.val.accZ_g1000 / 1000.0f;

    Serial.print("Force_N:");    Serial.print(force_N, 2);    Serial.print(", ");
    // Serial.print("Weight_kg:");  Serial.print(weight_kg, 3);  Serial.print(", ");
    Serial.print("AccX_g:");     Serial.print(ax, 3);         Serial.print(", ");
    Serial.print("AccY_g:");     Serial.print(ay, 3);         Serial.print(", ");
    Serial.print("AccZ_g:");     Serial.println(az, 3);
  } 
  else {
    fail_count++;
    if (fail_count >= 10) {
      resetI2CMaster();
      fail_count = 0;
    }
  }
  delay(10);
}