#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600
#define ONBOARD_VOLTAGE 4.655 // Arduino onboard voltage (true output of 5V pin)
#define VPIN 3 // voltage pin analog input
#define VOLTAGE_PERIOD 1000 // how often to print out voltage

MS5837 sensor;
bool pressureConnected = false;

float voltage;
unsigned long myTime;
unsigned long prevTime;
String pressuretag = "P:";
String voltagetag = "V:";
String printPressure = "";
String printVoltage = "";

// Make one attempt to initialize to the pressure sensor
void initPressureSensor(){
    Wire.end();
    Wire.begin();
    Wire.setWireTimeout(500, true);

    pressureConnected = sensor.init();

    if (pressureConnected)
        sensor.setModel(MS5837::MS5837_02BA);
}

void setup(){

    delay(100);
    pinMode(VPIN, OUTPUT);
    Serial.begin(BAUD_RATE);
    voltage = 0;
    prevTime = 0;

    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    // Initialize the pressure sensor
    initPressureSensor();
}

void loop(){
    // If pressure sensor is connected, read the pressure
    if (pressureConnected) {
        byte error = sensor.read();

        // If sensor.read timed out, mark pressure sensor as disconnected and clear the timeout flag
        if (error == 5) {
            pressureConnected = false;
            Wire.clearWireTimeoutFlag();
        }

        // If sensor.read was successful, print the pressure
        if (!error) {
            Serial.flush();
            printPressure = pressuretag + String(sensor.depth());
            Serial.println(printPressure);
        }

        // If sensor.read had an error but did not time out, try reading again in next loop
    }

    // If pressure sensor is disconnected, try to reinitalize it
    else {
        initPressureSensor();
    }

    // Print the voltage every VOLTAGE_PERIOD milliseconds
    myTime = millis();
    if(myTime - prevTime > VOLTAGE_PERIOD) {
      prevTime = myTime;

      voltage = analogRead(VPIN);
      voltage = voltage*ONBOARD_VOLTAGE/1023*5; // from datasheet, for analog to digital conversion
      printVoltage = voltagetag + String(voltage);
      Serial.println(printVoltage);
    }
}