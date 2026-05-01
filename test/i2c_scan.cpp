#include <Arduino.h>
#include <Wire.h>
#include "config.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("I2C scan starting...");

  // Initialize I2C using project's config pins and frequency
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_BUS_FREQUENCY_HZ);
  Wire.setClock(I2C_BUS_FREQUENCY_HZ);
  delay(100);

  int found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("Found device at 0x%02X\n", address);
      ++found;
    }
    delay(5);
  }

  if (found == 0) {
    Serial.println("No I2C devices found. Check wiring and pins.");
  } else {
    Serial.printf("Scan complete, %d device(s) found.\n", found);
  }
}

void loop() {
  delay(1000);
}
