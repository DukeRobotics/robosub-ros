#define humiditytag "H:"
#define temperaturetag "T:"
#define DHT11PIN 4  
#define SERVOPIN 9

DHT11 dht11(DHT11PIN);

float ONBOARD_VOLTAGE = 4.655;
bool servoMoved = false;
unsigned long servoTime;

Servo myservo;

// Initializes the servo for the marker dropper
initServo() {
    myservo.attach(SERVOPIN); 
    myservo.writeMicroseconds(1500); // 1500 micros = 90 write
}

// Opens and closes the marker dropper servo
callServo() {
    unsigned long currentTime = millis();

    if (Serial.available() > 0 && !servoMoved) {
        char state = Serial.read();
        if (Serial.available() > 0) {
            Serial.readString();
        }

        if (state == 'L') {
            myservo.writeMicroseconds(1250);  // 1200 = 90 degrees left  // 1250 micros =  20 write
            servoMoved = true;
            servoTime = currentTime;
        }
        if (state == 'R') {
            myservo.writeMicroseconds(1750);  // 1800 = 90 degrees right // 1750 micros = 160 write
            servoMoved = true;
            servoTime = currentTime;
        }
    }

    // return to default position after 1 second
    // if(servoMoved && myTime - servoTime > rateServo) {
    //     myservo.writeMicroseconds(1500);  // 1500 micros = 90 write
    //     servoMoved = false;
    // }
}


// Initializes the single temp/humidity sensor
initTempHumidity() {
    dht11.setDelay(0);
} 


// Reads the temp and humidity and prints it over serial
callTempHumidity() {
    int temperature = 0;
    int humidity = 0;
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    if (result == 0) {
      String printHumidity = humiditytag + String((float)humidity);
      Serial.println(printHumidity);

      String printTemp = temperaturetag + String((float)temperature * 1.8 + 32); // convert Celsius to Fahrenheit
      Serial.println(printTemp);
    }
}