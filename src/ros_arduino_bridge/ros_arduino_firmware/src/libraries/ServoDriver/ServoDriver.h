/***************************************************
  Library for 2 DC motor & 16 Servo driver

  by DOIT. http://www.doit.am
 ****************************************************/

#ifndef _ServoDriver_H
#define _ServoDriver_H
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define LED0_ON_L 0x6
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE


class ServoDriver {
 public:
  ServoDriver(uint8_t addr = 0x40);
  void begin(void);
  void reset(void);
  void setPWMFreq(float freq);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);

 private:
  uint8_t _i2caddr;
  void write8(uint8_t addr, uint8_t d);
  uint8_t read8(uint8_t addr);
};

#endif
