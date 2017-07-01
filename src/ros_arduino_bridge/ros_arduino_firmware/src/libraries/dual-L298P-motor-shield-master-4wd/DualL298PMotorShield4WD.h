#ifndef DualL298PMotorShield4WD_h
#define DualL298PMotorShield4WD_h

#include <Arduino.h>

class DualL298PMotorShield4WD
{
  public:  
    // CONSTRUCTORS
    DualL298PMotorShield4WD(); // Default pin selection.
    
    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setM3Speed(int speed); // Set speed for M3.
    void setM4Speed(int speed); // Set speed for M4.
    void setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed); // Set speed for both M1 and M2.
    
  private:
  
    // left motor
    static const unsigned char _M1DIR = 12;
    static const unsigned char _M2DIR = 7;
    static const unsigned char _M1PWM = 10;
    static const unsigned char _M2PWM = 6;
    
    // right motor
    static const unsigned char _M4DIR = 8;
    static const unsigned char _M3DIR = 13;
    static const unsigned char _M4PWM = 9;
    static const unsigned char _M3PWM = 11;
    
    //static const unsigned char _M1DIR = 4;
    //static const unsigned char _M2DIR = 7;
    //static const unsigned char _M1PWM = 5;
    //static const unsigned char _M2PWM = 6;
};

#endif