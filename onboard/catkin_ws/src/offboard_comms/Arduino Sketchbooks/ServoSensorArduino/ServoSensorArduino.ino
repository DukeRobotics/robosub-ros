// from L to R: data (pin 4), power (5V), ground
// from L to R: data (pin 4), power (5V), ground
#include <Servo.h>
#include <dht11.h>
#define DHT11PIN 4  // digital pin 4
#include <dht11.h>
#define DHT11PIN 4  // digital pin 4

dht11 DHT11;
dht11 DHT11;
Servo myservo;

unsigned long myTime;
boolean servoMoved = false;
unsigned long servoTime;
unsigned long rateServo = 1000;  // how much to wait before returning servo to default position
unsigned long rateTemp = 0;  // how often to print temp/humidity (no delay)
unsigned long prevTime = 0; 
String humiditytag = "H:";
String temperaturetag = "T:";
String printHumidity = "";
String printTemp = "";

unsigned long rateTemp = 0;  // how often to print temp/humidity (no delay)
unsigned long prevTime = 0; 
String humiditytag = "H:";
String temperaturetag = "T:";
String printHumidity = "";
String printTemp = "";


void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // servo pin 9
  myservo.writeMicroseconds(1500); // 1500 micros = 90 write

  myservo.writeMicroseconds(1500); // 1500 micros = 90 write

}

void loop() {
  myTime = millis();
  
  if (Serial.available() > 0 && !servoMoved) {
    char state = Serial.read();
    if (Serial.available() > 0) {
      Serial.readString();
    }

    if (state == 'L') {
        myservo.writeMicroseconds(1250);  // 1200 = 90 degrees left  // 1250 micros =  20 write
        myservo.writeMicroseconds(1250);  // 1200 = 90 degrees left  // 1250 micros =  20 write
        servoMoved = true;
        servoTime = myTime;
      }
      if (state == 'R') {
        myservo.writeMicroseconds(1750);  // 1800 = 90 degrees right // 1750 micros = 160 write
        myservo.writeMicroseconds(1750);  // 1800 = 90 degrees right // 1750 micros = 160 write
        servoMoved = true;
        servoTime = myTime;
      }
  }
  
  // return to default position after 1 second
  if(servoMoved && myTime - servoTime > rateServo) {
      myservo.writeMicroseconds(1500);  // 1500 micros = 90 write
      myservo.writeMicroseconds(1500);  // 1500 micros = 90 write
      servoMoved = false;
    }

  // printing humidity and temperature
  if(myTime - prevTime >= rateTemp) {
    prevTime = myTime;
    int chk = DHT11.read(DHT11PIN);

    printHumidity = humiditytag + String((float)DHT11.humidity);
    Serial.println(printHumidity);

    printTemp = temperaturetag + String((float)DHT11.temperature * 1.8 + 32);
    Serial.println(printTemp);
  }
    }

  // printing humidity and temperature
  if(myTime - prevTime >= rateTemp) {
    prevTime = myTime;
    int chk = DHT11.read(DHT11PIN);

    printHumidity = humiditytag + String((float)DHT11.humidity);
    Serial.println(printHumidity);

    printTemp = temperaturetag + String((float)DHT11.temperature * 1.8 + 32);
    Serial.println(printTemp);
  }
}