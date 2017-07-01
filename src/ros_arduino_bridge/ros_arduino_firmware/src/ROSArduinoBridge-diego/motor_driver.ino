/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#ifdef USE_BASE

#if defined POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"

/* Create the motor driver object */
DualVNH5019MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"

/* Create the motor driver object */
DualMC33926MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined L298P
#include "DualL298PMotorShield.h"

/* Create the motor driver object */
DualL298PMotorShield drive;
/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
//  Serial.println("setMotorSpeeds.output");
//  Serial.println(leftSpeed);
//  Serial.println("setMotorSpeeds.output");
//  Serial.println(rightSpeed);
}
#elif defined L298P_4WD
#include "DualL298PMotorShield4WD.h"

/* Create the motor driver object */
DualL298PMotorShield4WD drive;
/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}


/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == 1) drive.setM1Speed(spd);
  else if (i == 2) drive.setM2Speed(spd);
  else if (i==3)  drive.setM3Speed(spd);
  else  drive.setM4Speed(spd);
}

#ifdef L298P
// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int leftSpeed){
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);

//  Serial.println("setMotorSpeeds.output");
//  Serial.println(leftSpeed);
//  Serial.println("setMotorSpeeds.output");
//  Serial.println(rightSpeed);
}
#endif
#ifdef L298P_4WD
// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed_1, int leftSpeed_2, int rightSpeed_1, int rightSpeed_2){
  setMotorSpeed(1, leftSpeed_1);
  setMotorSpeed(3, rightSpeed_1);
  setMotorSpeed(2, leftSpeed_2);
  setMotorSpeed(4, rightSpeed_2);
//  Serial.println("setMotorSpeeds.output");
//  Serial.println(leftSpeed);
//  Serial.println("setMotorSpeeds.output");
//  Serial.println(rightSpeed);
}
#endif
#else
#error A motor driver must be selected!
#endif

#endif
