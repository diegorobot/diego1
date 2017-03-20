/***************************************************
  Library for 2 DC motor & 16 Servo driver

  by DOIT. http://www.doit.am
 ****************************************************/

#include <ServoDriver.h>
#include <Wire.h>
#if defined(__AVR__)
 #define WIRE Wire
#elif defined(CORE_TEENSY)
 #define WIRE Wire
#else // Arduino Due
 #define WIRE Wire
#endif


ServoDriver::ServoDriver(uint8_t addr) {
  _i2caddr = addr;
}

void ServoDriver::begin(void) {
 WIRE.begin();
 reset();
}

void ServoDriver::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}


void ServoDriver::setPWMFreq(float freq) {

  freq *= 0.9;
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;

  uint8_t prescale = floor(prescaleval + 0.5);

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10;
  write8(PCA9685_MODE1, newmode);
  write8(PCA9685_PRESCALE, prescale);
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);

}

void ServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {

  WIRE.beginTransmission(_i2caddr);
  WIRE.write(LED0_ON_L+4*num);
  WIRE.write(on);
  WIRE.write(on>>8);
  WIRE.write(off);
  WIRE.write(off>>8);
  WIRE.endTransmission();
}

uint8_t ServoDriver::read8(uint8_t addr) {
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.endTransmission();

  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return WIRE.read();
}

void ServoDriver::write8(uint8_t addr, uint8_t d) {
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();

}
