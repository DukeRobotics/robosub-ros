#include "MultiplexedBasicESC.h"

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

MultiplexedBasicESC::~MultiplexedBasicESC(){detach();}

void MultiplexedBasicESC::initialize(Adafruit_PWMServoDriver *multiplexer){driver.initialize(multiplexer);}

void MultiplexedBasicESC::attach(uint8_t pin){driver.attach(pin);}

bool MultiplexedBasicESC::attached(){return driver.attached();}

void MultiplexedBasicESC::detach(){driver.detach();}

void MultiplexedBasicESC::write(int8_t speed){writeMicroseconds(speed+31);} //apply offset of +31 for oogway, remove for cthulhu

void MultiplexedBasicESC::writeMicroseconds(uint16_t uS){driver.writeMicroseconds(uS);}
