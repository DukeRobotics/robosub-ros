// from L to R: data (pin 4), power (5V), ground
#include <Servo.h>
#include <DHT11.h>
#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#define DHT11PIN 4  // digital pin 4

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600
#define ONBOARD_VOLTAGE 4.655 // Arduino onboard voltage (true output of 5V pin)
#define VPIN 3 // voltage pin analog input
#define VOLTAGE_PERIOD 100 // how often to print out voltage

#define rateServo 1000  // how much to wait before returning servo to default position
#define humiditytag "H:"
#define temperaturetag "T:"
#define printHumidity ""
#define printTemp ""

unsigned long myTimeP;
boolean servoMoved = false;
unsigned long servoTime;
unsigned long rateTemp = 0;  // how often to print temp/humidity (no delay)
unsigned long prevTimeP = 0;

DHT11 dht11(DHT11PIN);
Servo myservo;

MS5837 sensor;
bool pressureConnected = false;

float voltage;
unsigned long myTimeV;
unsigned long prevTimeV;
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

void setup() {
  Serial.begin(BAUD_RATE);
  myservo.attach(9);  // servo pin 9
  myservo.writeMicroseconds(1500); // 1500 micros = 90 write
  pinMode(VPIN, OUTPUT);
  voltage = 0;
  prevTime = 0;
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  // Initialize the pressure sensor
  initPressureSensor();
}

void loop() {
  myTimeP = millis();

  if (Serial.available() > 0 && !servoMoved) {
    char state = Serial.read();
    if (Serial.available() > 0) {
      Serial.readString();
    }

    if (state == 'L') {
        myservo.writeMicroseconds(1250);  // 1200 = 90 degrees left  // 1250 micros =  20 write
        servoMoved = true;
        servoTime = myTimeP;
      }
      if (state == 'R') {
        myservo.writeMicroseconds(1750);  // 1800 = 90 degrees right // 1750 micros = 160 write
        servoMoved = true;
        servoTime = myTimeP;
      }
  }

  // return to default position after 1 second
  if(servoMoved && myTimeP - servoTime > rateServo) {
      myservo.writeMicroseconds(1500);  // 1500 micros = 90 write
      servoMoved = false;
    }

  // printing humidity and temperature
  if(myTimeP - prevTimeP >= rateTemp) {
    prevTimeP = myTimeP;
    int temperature = 0;
    int humidity = 0;
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    if (result == 0) {
      printHumidity = humiditytag + String((float)humidity);
      Serial.println(printHumidity);

      printTemp = temperaturetag + String((float)temperature * 1.8 + 32); // convert Celsius to Fahrenheit
      Serial.println(printTemp);
    }
    // else: error reading sensor
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
  myTimeV = millis();
  if(myTimeV - prevTimeV > VOLTAGE_PERIOD) {
    prevTimeV = myTimeV;

    voltage = analogRead(VPIN);
    voltage = voltage*ONBOARD_VOLTAGE/1023*5; // from datasheet, for analog to digital conversion
    printVoltage = voltagetag + String(voltage);
    Serial.println(printVoltage);
  }
}


