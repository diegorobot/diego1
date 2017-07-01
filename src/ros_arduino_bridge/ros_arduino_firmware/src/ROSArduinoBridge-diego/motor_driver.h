/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

void initMotorController();
void setMotorSpeed(int i, int spd);
#ifdef L298P
void setMotorSpeeds(int leftSpeed, int rightSpeed);
#endif
#ifdef L298P_4WD
void setMotorSpeeds(int leftSpeed_1, int leftSpeed_2, int rightSpeed_1, int rightSpeed_2);
#endif
