#ifndef Servos_h
#define Servos_h

#include <Arduino.h>
#include "MultiplexedObject.h"
#include "Adafruit_PWMServoDriver.h"

class MultiplexedServo:public MultiplexedObject{
public:
    MultiplexedServo(Adafruit_PWMServoDriver *, int);
    void initialise();
    void run(int);
};

#endif
