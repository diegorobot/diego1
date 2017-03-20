#include "DualL298PMotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

DualL298PMotorShield::DualL298PMotorShield()
{
  //Pin map

}


// Public Methods //////////////////////////////////////////////////////////////
void DualL298PMotorShield::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
  
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualL298PMotorShield::setM1Speed(int speed)
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
void DualL298PMotorShield::setM2Speed(int speed)
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

// Set speed for motor 1 and 2
void DualL298PMotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}