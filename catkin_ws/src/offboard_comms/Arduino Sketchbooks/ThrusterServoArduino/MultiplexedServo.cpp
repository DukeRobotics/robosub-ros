#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"
#include "MultiplexedObject.h"

MultiplexedServo::MultiplexedServo(Adafruit_PWMServoDriver *_multiplexer,int _num):MultiplexedObject(_multiplexer){
    num=_num;
}

void MultiplexedServo::initialise(){
    attach(num);
    run(90);
}

void MultiplexedServo::run(int power){
    float pulse=map(power, 0, 180, 650, 2450);
    writeMicroseconds(pulse);
}
