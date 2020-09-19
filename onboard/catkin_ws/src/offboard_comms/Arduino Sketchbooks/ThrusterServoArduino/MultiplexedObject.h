#ifndef MultiplexedObject_h
#define MultiplexedObject_h

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"
#include <Servo.h>

class MultiplexedObject{
protected:
    int num;
    Adafruit_PWMServoDriver *multiplexer;
    bool attached;
public:
    MultiplexedObject(Adafruit_PWMServoDriver *, int);
    ~MultiplexedObject();
    void attach(int);
    void detach();
    void writeMicroseconds(int secs);
    virtual void initialise() = 0;
    virtual void run(int) = 0;
};

#endif
