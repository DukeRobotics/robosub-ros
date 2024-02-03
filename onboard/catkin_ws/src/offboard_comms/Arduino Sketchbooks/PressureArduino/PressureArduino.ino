#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600
#define ONBOARD_VOLTAGE 4.776 // 4.776 is arduino onboard voltage (true output of 5V pin)
#define VPIN 3 // voltage pin analog input
#define VOLTAGE_PERIOD 1000 // how often to print out voltage

float voltage;
unsigned long myTime;
unsigned long prevTime;
String pressuretag = "P:";
String voltagetag = "V:";
String printPressure = "";
String printVoltage = "";

void setup(){

    delay(100);
    pinMode(VPIN, OUTPUT);
    Serial.begin(BAUD_RATE);
    voltage = 0;
    prevTime = 0;

    Wire.begin();
    Wire.setWireTimeout(1000, true);

    while (!sensor.init()) {
        delay(1000);
        break;
    }
    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
    sensor.setModel(MS5837::MS5837_30BA);
}

void loop(){
    byte response = sensor.read();
    if (response == 5) {
      Serial.println("Timeout; Exited senor.read");
      delay(1000);
      while (!sensor.init()) {
        delay(1000);
        break;
      }
      Serial.println("Reinitialized sensor");
      return;
    } 
    Serial.flush();
    printPressure = pressuretag + String(sensor.depth());
    Serial.println(printPressure);

//     myTime = millis();
//     if(myTime - prevTime > VOLTAGE_PERIOD) {
//       prevTime = myTime;

//       voltage = analogRead(VPIN);
//       voltage = voltage*ONBOARD_VOLTAGE/1023*5; // from datasheet, for analog to digital conversion
//       printVoltage = voltagetag + String(voltage);
//       Serial.println(printVoltage);
//     }


}