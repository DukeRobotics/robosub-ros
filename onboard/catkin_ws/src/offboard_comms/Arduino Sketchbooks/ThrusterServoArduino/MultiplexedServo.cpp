#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"

MultiplexedServo::MultiplexedServo(){
  is_attached = false;
}

MultiplexedServo::~MultiplexedServo(){
  detach();
}

void MultiplexedServo::initialize(Adafruit_PWMServoDriver *_multiplexer){
  multiplexer = _multiplexer;
}

void MultiplexedServo::attach(uint8_t _pin){
  if(is_attached){
    detach();
  }
  is_attached = true;
  pin = _pin;
  multiplexer->setPWMFreq(250);
}

bool MultiplexedServo::attached(){return is_attached;}

void MultiplexedServo::detach(){
  is_attached = false;
  multiplexer->setPin(pin, 0, false);
}

void MultiplexedServo::write(uint8_t angle){
  writeMicroseconds(map(angle, 0, 180, 650, 2450));
}

void MultiplexedServo::writeMicroseconds(uint16_t uS){
  if(is_attached) {
    multiplexer->setPin(pin, map(uS, 1100, 1900, 1000, 1720), false);
  }
}
