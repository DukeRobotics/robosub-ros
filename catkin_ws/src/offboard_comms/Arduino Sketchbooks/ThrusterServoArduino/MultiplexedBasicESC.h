#ifndef BasicESC_h
#define BasicESC_h

#include <Arduino.h>
#include "MultiplexedObject.h"
#include "Adafruit_PWMServoDriver.h"

class MultiplexedBasicESC:public MultiplexedObject{
public:
    MultiplexedBasicESC(Adafruit_PWMServoDriver *, int);
    void initialise();
    void run(int);
};

#endif
