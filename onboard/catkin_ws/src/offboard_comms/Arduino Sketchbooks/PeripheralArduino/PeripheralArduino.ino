// from L to R: data (pin 4), power (5V), ground
#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "DHT11.h"
#include "MS5837.h"
#include "tempHumidity.h"

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600
#define VPIN 3 // voltage pin analog input
#define VOLTAGE_PERIOD 100 // how soften to print out voltage
#define RATESERVO 1000  // how much to wait before returning servo to default position
#define RATETEMP 1000 // how often to print temp/humidity
#define PRESSURETAG "P:"
#define VOLTAGETAG "V:"

extern float ONBOARD_VOLTAGE; // Arduino onboard voltage (true output of 5V pin)

MS5837 sensor;
bool pressureConnected = false;

unsigned long currentTime;
unsigned long prevTimePressure = 0;
float voltage;
unsigned long prevTimeVoltage = 0;


// Make one attempt to initialize to the pressure sensor
void initPressureSensor(){
  Wire.end();
  Wire.begin();
  Wire.setWireTimeout(500, true);

  pressureConnected = sensor.init();

  if (pressureConnected)
      sensor.setModel(MS5837::MS5837_02BA);
}

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(VPIN, OUTPUT);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  // Initialize the servo
  initServo();
  // Initialize the pressure sensor
  initPressureSensor();
  // Initialize the temp/humidity sensor
  initTempHumidity();
}

void loop() {
  currentTime = millis();

  // Call the marker dropper servo
  callServo();

  // printing humidity and temperature
  if(currentTime - prevTimePressure >= RATETEMP) {
    prevTimePressure = currentTime;
    callTempHumidity();
  }

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
          String printPressure = PRESSURETAG + String(sensor.depth());
          Serial.println(printPressure);
      }

      // If sensor.read had an error but did not time out, try reading again in next loop
  }

  // If pressure sensor is disconnected, try to reinitalize it
  else {
      initPressureSensor();
  }

  // Print the voltage every VOLTAGE_PERIOD milliseconds
  currentTime = millis();
  if(currentTime - prevTimeVoltage > VOLTAGE_PERIOD) {
    prevTimeVoltage = currentTime;

    voltage = analogRead(VPIN);
    voltage = voltage*ONBOARD_VOLTAGE/1023*5; // from datasheet, for analog to digital conversion
    String printVoltage = VOLTAGETAG + String(voltage);
    Serial.println(printVoltage);
  }
}