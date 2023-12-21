#ifndef MULTIPLEXEDBASICESC_H
#define MULTIPLEXEDBASICESC_H

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"

class MultiplexedBasicESC{
private:
  MultiplexedServo driver;
public:
  ~MultiplexedBasicESC();
  void initialize(Adafruit_PWMServoDriver *);
  void attach(uint8_t);
  bool attached();
  void detach();
  void write(uint16_t);
  void writeMicroseconds(uint16_t);
};

#endif // MULTIPLEXEDBASICESC_H
