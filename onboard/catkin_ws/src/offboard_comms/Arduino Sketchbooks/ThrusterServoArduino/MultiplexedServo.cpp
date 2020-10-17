#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedServo.h"

MultiplexedServo::MultiplexedServo(Adafruit_PWMServoDriver *_multiplexer):multiplexer(_multiplexer){
  is_attached = false;
}

MultiplexedServo::~MultiplexedServo(){
  detach();
}

void MultiplexedServo::attach(uint8_t _pin){
  attach(_pin, 544, 2400);
}

void MultiplexedServo::attach(uint8_t _pin, uint16_t _out_min, uint16_t _out_max){
  attach(_pin, _out_min, _out_max, 0, 180);
}

void MultiplexedServo::attach(uint8_t _pin, uint16_t _out_min, uint16_t _out_max, int16_t _in_min, int16_t _in_max){
  if(is_attached)
      detach();
  is_attached = true;
  pin = _pin;
  in_min = _in_min;
  in_max = _in_max;
  out_min = _out_min;
  out_max = _out_max;
  multiplexer->setPWMFreq(250);
}

bool MultiplexedServo::attached(){return is_attached;}

void MultiplexedServo::detach(){
  is_attached = false;
  multiplexer->setPin(pin, 0, false);
}

int16_t MultiplexedServo::read(){return last_value;}

void MultiplexedServo::write(int16_t angle){
 last_value = angle;
 writeMicroseconds(map(angle, in_min, in_max, out_min, out_max));
}

void MultiplexedServo::writeMicroseconds(uint16_t uS){
  if(is_attached) {
      multiplexer->setPin(pin, map(uS, 1100, 1900, 1000, 1720), false);
  }
}
