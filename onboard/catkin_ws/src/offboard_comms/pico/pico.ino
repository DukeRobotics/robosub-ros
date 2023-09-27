#include "MS5837.h"

#define BAUD_RATE 9600

MS5837 pressure_sensor;

void setup() {
  Serial1.begin(BAUD_RATE);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  while (!pressure_sensor.init()) {
        Serial.println("Pressure sensor not initialized. Will attempt every second until found.");
        delay(1000);
        break;
    }

    pressure_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    Serial.println("Pressure sensor initialized.");
  digitalWrite(LED_BUILTIN, LOW);



}

void loop() {
  pressure_sensor.read();
  Serial.println(pressure_sensor.depth());
}
