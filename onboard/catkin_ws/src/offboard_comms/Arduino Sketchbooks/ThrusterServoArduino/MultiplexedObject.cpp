#include <Arduino.h>
#include <Wire.h>
#include "MultiplexedObject.h"

MultiplexedObject::MultiplexedObject(Adafruit_PWMServoDriver *_multiplexer):multiplexer(_multiplexer){
    attached = false;
}

MultiplexedObject::~MultiplexedObject(){
    detach(num);
}

void MultiplexedObject::attach(int _num){
    // make sure it isn't trying to run multiple pins
    if(attached)
        detach(num);
    attached = true;
    num = _num;
    multiplexer->setPWMFreq(250);
}

void MultiplexedObject::detach(int _num){
    attached = false;
    multiplexer->setPin(num,0,false);
}

void MultiplexedObject::writeMicroseconds(int secs){
    if (attached)
    {
        int mappedPulse = map(secs, 1100, 1900, 1000, 1720);
        multiplexer->setPin(num, mappedPulse, false);
    }
}
