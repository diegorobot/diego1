#include "DualL298PMotorShield4WD.h"

// Constructors ////////////////////////////////////////////////////////////////

DualL298PMotorShield4WD::DualL298PMotorShield4WD()
{
  //Pin map

}


// Public Methods //////////////////////////////////////////////////////////////
void DualL298PMotorShield4WD::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
  pinMode(_M3DIR,OUTPUT);
  pinMode(_M3PWM,OUTPUT);
  pinMode(_M4DIR,OUTPUT);
  pinMode(_M4PWM,OUTPUT);  
  
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualL298PMotorShield4WD::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  if (reverse)
  {
    digitalWrite(_M1DIR,LOW);
    analogWrite(_M1PWM, speed);
  }
  else
  {
    digitalWrite(_M1DIR,HIGH);
    analogWrite(_M1PWM, speed);
  }    
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualL298PMotorShield4WD::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  if (reverse)
  {
    digitalWrite(_M2DIR,LOW);
    analogWrite(_M2PWM, speed);
  }
  else
  {
    digitalWrite(_M2DIR,HIGH);
    analogWrite(_M2PWM, speed);
  }
}

// Set speed for motor 3, speed is a number betwenn -400 and 400
void DualL298PMotorShield4WD::setM3Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  if (reverse)
  {
    digitalWrite(_M3DIR,LOW);
    analogWrite(_M3PWM, speed);
  }
  else
  {
    digitalWrite(_M3DIR,HIGH);
    analogWrite(_M3PWM, speed);
  }
}

// Set speed for motor 4, speed is a number betwenn -400 and 400
void DualL298PMotorShield4WD::setM4Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
  if (reverse)
  {
    digitalWrite(_M4DIR,LOW);
    analogWrite(_M4PWM, speed);
  }
  else
  {
    digitalWrite(_M4DIR,HIGH);
    analogWrite(_M4PWM, speed);
  }
}

// Set speed for motor 1, 2, 3, 4
void DualL298PMotorShield4WD::setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
  setM4Speed(m4Speed);  
}