#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
const int BAUD_RATE = 9600;

const int aPin = 3; // voltage pin
float voltage;
float pressure;
unsigned long myTime;
unsigned long prevTime;
unsigned long rate = 1000;  // how often to print out voltage
String pressuretag = "P:";  
String voltagetag = "V:";
String printPressure = "";
String printVoltage = "";

void setup(){

    delay(100);
    pinMode(aPin, OUTPUT);
    Serial.begin(9600);
    voltage = 0;
    prevTime = 0;

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
    myTime = millis();
   
    if(myTime - prevTime > rate) {
      prevTime = myTime;
   
      voltage = analogRead(aPin);                    
      voltage = voltage*4.776/1023*5; //4.776 is arduino onboard voltage (4.826 for Carson's laptop)
      printVoltage = voltagetag + String(voltage);
      Serial.println(printVoltage);
    }
   
    pressure = sensor.depth();
    printPressure = pressuretag + String(pressure);
    Serial.println(printPressure);
}

