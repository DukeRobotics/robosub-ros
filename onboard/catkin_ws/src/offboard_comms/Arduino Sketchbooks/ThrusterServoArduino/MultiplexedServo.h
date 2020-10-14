#ifndef MULTIPLEXEDSERVO_H
#define MULTIPLEXEDSERVO_H

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"

class MultiplexedServo{
private:
  uint8_t pin;
  bool is_attached;
  uint16_t last_value;
  int16_t in_min;
  int16_t in_max;
  uint16_t out_min;
  uint16_t out_max;
  Adafruit_PWMServoDriver *multiplexer;
public:
  MultiplexedServo(Adafruit_PWMServoDriver *);
  ~MultiplexedServo();
  void attach(uint8_t);
  void attach(uint8_t, uint16_t, uint16_t);
  void attach(uint8_t, uint16_t, uint16_t, int16_t, int16_t);
  bool attached();
  void detach();
  uint16_t read();
  void write(uint16_t);
  void writeMicroseconds(uint16_t);
};

#endif // MULTIPLEXEDSERVO_H
