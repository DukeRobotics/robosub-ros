#define DHT11COMPUTERPIN 2  // digital pin 2 for temp/humidity sensor in computer capsule
#define DHT11BATTERYPIN 4  // digital pin 4 for temp/humidity sensor in battery capsule

#define humiditytagComputer "HC:"
#define temperaturetagComputer "TC:"
#define humiditytagBattery "HB:"
#define temperaturetagBattery "TB:"

DHT11 dht11Computer(DHT11COMPUTERPIN);
DHT11 dht11Battery(DHT11BATTERYPIN);

float ONBOARD_VOLTAGE = 4.96;

// Empty function to account for servo on oogway arduino
initServo() {}

// Empty function to account for servo on oogway arduino
callServo() {}

// Initializes the two temp/humidity sensors
initTempHumidity() {
    dht11Computer.setDelay(0);
    dht11Battery.setDelay(0);
}

// Reads the temp and humidity and prints it over serial
callTempHumidity() {
    int temperatureComputer = 0;
    int humidityComputer = 0;
    int resultComputer = dht11Computer.readTemperatureHumidity(temperatureComputer, humidityComputer);

    int temperatureBattery = 0;
    int humidityBattery = 0;
    int resultBattery = dht11Battery.readTemperatureHumidity(temperatureBattery, humidityBattery);

    if (resultComputer == 0) {
        String printHumidityComputer = humiditytagComputer + String((float)humidityComputer);
        Serial.println(printHumidityComputer);

        String printTempComputer = temperaturetagComputer + String((float)temperatureComputer * 1.8 + 32); // convert Celsius to Fahrenheit
        Serial.println(printTempComputer);
    }

    if(resultBattery == 0) {
        String printHumidityBattery = humiditytagBattery + String((float)humidityBattery);
        Serial.println(printHumidityBattery);

        String printTempBattery = temperaturetagBattery + String((float)temperatureBattery * 1.8 + 32); // convert Celsius to Fahrenheit
        Serial.println(printTempBattery);
    }
}