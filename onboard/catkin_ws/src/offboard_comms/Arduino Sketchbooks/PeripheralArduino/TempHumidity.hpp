#include <Arduino.h>
#include <DHT11.h>

class TempHumidity {
    private:
        int pinNum;
        DHT11 dht11;
        string humidityTag;
        string tempTag;
    public:
        TempHumidity(int pinNum, string tagPrefix) : pinNum(pinNum) {
            humidityTag = tagPrefix + "_H";
            tempTag = tagPrefix + "_T";
            
            dht11(pinNum);
            dht11.setDelay(0);
            
        }

        void callTempHumidity() {
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
}