#ifndef MULTIPLEXEDBASICESC_H
#define MULTIPLEXEDBASICESC_H

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"

class MultiplexedBasicESC: public MultiplexedServo{
public:
  void attach(uint8_t);
  void write(int8_t);
};

#endif // MULTIPLEXEDBASICESC_H
