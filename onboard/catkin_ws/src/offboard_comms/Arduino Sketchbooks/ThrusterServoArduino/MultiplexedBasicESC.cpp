#include "MultiplexedBasicESC.h"

void MultiplexedBasicESC::attach(uint8_t _pin){
  MultiplexedServo::attach(_pin); 
  write(0);
}

void MultiplexedBasicESC::write(int8_t speed){
  writeMicroseconds(map(speed, -128, 128, 1100, 1900));
}
