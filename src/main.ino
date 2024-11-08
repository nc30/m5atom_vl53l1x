/**
 * This example contains a basic setup to start the distance measurement of the VL53L1X
 */
#include <EEPROM.h>
#include "VL53L1X_ULD.h"
#include "M5Atom.h"

#define GPIO_SDA 25
#define GPIO_SCL 21
#define GPIO_OUT 22
//#define GPIO_A 23
#define GPIO_A 39  // Atomのボタン
#define GPIO_B 33
// #define GPIO_B 39

VL53L1X_ULD sensor;

uint16_t scales[15] = {
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  12,
  18,
  24,
  30
};

uint8_t cursor;

uint8_t buttons[2] = {GPIO_A, GPIO_B};
uint8_t btncnt[2] = {0, 0};


void setup() {
  Serial.begin(115200);
  Wire.begin(GPIO_SDA, GPIO_SCL);
  pinMode(GPIO_OUT, OUTPUT);
  pinMode(GPIO_A, INPUT_PULLUP);
  pinMode(GPIO_B, INPUT_PULLUP);

  if(EEPROM.begin(32)) {
    Serial.println("EEPROM ready");
  }

  M5.begin(true, false, true);

  VL53L1_Error status = sensor.Begin();
  if (status != VL53L1_ERROR_NONE) {
    Serial.println("Could not initialize the sensor, error code: " + String(status));
    while (1) {
      vTaskDelay(1000 / portTICK_RATE_MS);
      ESP.restart();
    }
  }
  Serial.println("Sensor initialized");

  sensor.StartRanging();

  cursor = EEPROM.read(0);
  if(cursor > 14) {
    cursor = 3;
  }

  Serial.println("cursor is "+String(cursor));

  xTaskCreate(vTaskButton, "vTaskButton", 2048, NULL, 2, NULL);
  xTaskCreate(vTaskSensor, "vTaskSensor", 2048, NULL, 1, NULL);
}


void vTaskButton(void *param) {
  uint8_t i;
  bool press;

  while (1) {
    for(i=0;i<2;i++){
      press = digitalRead(buttons[i]) == LOW;

      if(press && btncnt[i] == 0) {
        btncnt[i] = 1;
        continue;
      } 

      if(btncnt[i] > 2 && !press){
        btncnt[i] = 0;
        if(i == 0){
          if(cursor < 14){
            EEPROM.write(0, ++cursor);
            EEPROM.commit();
            // Serial.println("cursor "+String(cursor));
          }
        }else if(cursor > 0){
          EEPROM.write(0, --cursor);
          EEPROM.commit();
          // Serial.println("cursor "+String(cursor));
        }
        continue;
      }

      if(btncnt[i] > 0) {
        btncnt[i]++;
      }
    }

    vTaskDelay(50 / portTICK_RATE_MS);
  }
}

void vTaskSensor(void *param) {
  uint8_t dataReady = false;
  uint8_t i;

  while(1) {
    dataReady = 0;

    while(!dataReady) {
      sensor.CheckForDataReady(&dataReady);
      vTaskDelay(5 / portTICK_RATE_MS);
    }

    uint16_t distance;
    sensor.GetDistanceInMm(&distance);
    // Serial.println("Distance in mm: " + String(distance));
    sensor.ClearInterrupt();

    if (distance <= scales[cursor]){
      for(i=15;i<25;i++){
        M5.dis.drawpix(i, 0xfff000);
      }
      digitalWrite(GPIO_OUT, LOW);
    }else{
      for(i=15;i<25;i++){
        M5.dis.drawpix(i, 0xff0000);
      }
      digitalWrite(GPIO_OUT, HIGH);
    }

    for(i=0;i<15;i++){
      if(distance > scales[i]) {
        if(i == cursor){
          M5.dis.drawpix(i, 0x00ff00);
        }else{
          M5.dis.drawpix(i, 0x0000ff);
        }
      }else{
        M5.dis.drawpix(i, 0x000000);
      }
    }

    M5.update();

    vTaskDelay(50 / portTICK_RATE_MS);
  }
}

void loop() {
  return;
}
