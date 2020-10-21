#ifndef MULTIPLEXEDSERVO_H
#define MULTIPLEXEDSERVO_H

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"

class MultiplexedServo{
private:
  uint8_t pin;
  bool is_attached;
  Adafruit_PWMServoDriver *multiplexer;
public:
  MultiplexedServo();
  ~MultiplexedServo();
  void initialize(Adafruit_PWMServoDriver *);
  void attach(uint8_t);
  bool attached();
  void detach();
  void write(uint8_t);
  void writeMicroseconds(uint16_t);
};

#endif // MULTIPLEXEDSERVO_H
