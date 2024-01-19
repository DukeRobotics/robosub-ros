#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600

void setup(){

    delay(100);
    Serial.begin(BAUD_RATE);

    Wire.begin();

    while (!sensor.init()) {
        delay(1000);
        break;
    }

    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop(){

    sensor.read();
    Serial.flush();
    Serial.println(sensor.depth());
}