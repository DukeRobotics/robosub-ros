#include <Arduino.h>
#include <Wire.h>
#include "MultiplexedObject.h"

MultiplexedObject::MultiplexedObject(Adafruit_PWMServoDriver *_multiplexer, int _num):multiplexer(_multiplexer), num(_num){
    attached = false;
}

MultiplexedObject::~MultiplexedObject(){
    detach();
}

void MultiplexedObject::attach(int _num){
    // Make sure it isn't trying to run multiple pins
    if(attached)
        detach();
    attached = true;
    num = _num;
    multiplexer->setPWMFreq(250);
}

void MultiplexedObject::detach(){
    attached = false;
    multiplexer->setPin(num, 0, false);
}

void MultiplexedObject::writeMicroseconds(int secs){
    if (attached){
        int mappedPulse = map(secs, 1100, 1900, 1000, 1720);
        multiplexer->setPin(num, mappedPulse, false);
    }
}
