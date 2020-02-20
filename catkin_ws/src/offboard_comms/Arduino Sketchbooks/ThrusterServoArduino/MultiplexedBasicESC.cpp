#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include "MultiplexedObject.h"

MultiplexedBasicESC::MultiplexedBasicESC(Adafruit_PWMServoDriver *_multiplexer,int _num):MultiplexedObject(_multiplexer){
    num=_num;
}

void MultiplexedBasicESC::initialise(){
    attach(num);
    run(0);
}

void MultiplexedBasicESC::run(int power){
    float pulse=map(power, -128, 128, 1100, 1900);
    writeMicroseconds(pulse);
}
