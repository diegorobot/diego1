/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

#ifdef L298P_4WD
SetPointInfo leftPID_h, rightPID_h;
#endif

/* PID Parameters */
int Kp = 10;
int Kd = 12;
int Ki = 0;
int Ko = 50;

int left_Kp=Kp;
int left_Kd=Kd;
int left_Ki=Ki;
int left_Ko=Ko;

int right_Kp=Kp;
int right_Kd=Kd;
int right_Ki=Ki;
int right_Ko=Ko;

#ifdef L298P_4WD

int left_h_Kp=Kp;
int left_h_Kd=Kd;
int left_h_Ki=Ki;
int left_h_Ko=Ko;

int right_h_Kp=Kp;
int right_h_Kd=Kd;
int right_h_Ki=Ki;
int right_h_Ko=Ko;

#endif


unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
#ifdef L298P_4WD
   leftPID_h.TargetTicksPerFrame = 0.0;
   leftPID_h.Encoder = readEncoder(LEFT_H);
   leftPID_h.PrevEnc = leftPID_h.Encoder;
   leftPID_h.output = 0;
   leftPID_h.PrevInput = 0;
   leftPID_h.ITerm = 0;

   rightPID_h.TargetTicksPerFrame = 0.0;
   rightPID_h.Encoder = readEncoder(RIGHT_H);
   rightPID_h.PrevEnc = rightPID_h.Encoder;
   rightPID_h.output = 0;
   rightPID_h.PrevInput = 0;
   rightPID_h.ITerm = 0;
#endif   
}

/* PID routine to compute the next motor commands */
void dorightPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input =  p->Encoder-p->PrevEnc ;
  Perror = p->TargetTicksPerFrame - input;

//  Serial.println("right input:");
//  Serial.println(input);
//  Serial.println("right TargetTicksPerFrame:");
//  Serial.println(p->TargetTicksPerFrame);

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (right_Kp * Perror - right_Kd * (input - p->PrevInput) + p->ITerm) / right_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += left_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
//  Serial.println("right output:");
//  Serial.println(p->output);
}

#ifdef L298P_4WD
/* PID routine to compute the next motor commands */
void dorightPID_h(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input =  p->Encoder-p->PrevEnc ;
  Perror = p->TargetTicksPerFrame - input;

//  Serial.println("right input:");
//  Serial.println(input);
//  Serial.println("right TargetTicksPerFrame:");
//  Serial.println(p->TargetTicksPerFrame);

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (right_h_Kp * Perror - right_h_Kd * (input - p->PrevInput) + p->ITerm) / right_h_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += left_h_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
//  Serial.println("right output:");
//  Serial.println(p->output);
}
#endif

/* PID routine to compute the next motor commands */
void doleftPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder-p->PrevEnc ;
  Perror = p->TargetTicksPerFrame - input;

//  Serial.println("left input:");
//  Serial.println(input);
//  Serial.println("left TargetTicksPerFrame:");
//  Serial.println(p->TargetTicksPerFrame);

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (left_Kp * Perror - left_Kd * (input - p->PrevInput) + p->ITerm) / left_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += left_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
//  Serial.println("left output:");
//  Serial.println(p->output);
}
#ifdef L298P_4WD

/* PID routine to compute the next motor commands */
void doleftPID_h(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder-p->PrevEnc ;
  Perror = p->TargetTicksPerFrame - input;

//  Serial.println("left input:");
//  Serial.println(input);
//  Serial.println("left TargetTicksPerFrame:");
//  Serial.println(p->TargetTicksPerFrame);

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (left_h_Kp * Perror - left_h_Kd * (input - p->PrevInput) + p->ITerm) / left_h_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += left_h_Ki * Perror;

  p->output = output;
  p->PrevInput = input;
//  Serial.println("left output:");
//  Serial.println(p->output);
}
#endif

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder =readEncoder(LEFT);
  rightPID.Encoder =readEncoder(RIGHT);
#ifdef L298P_4WD 
  leftPID_h.Encoder =readEncoder(LEFT_H);
  rightPID_h.Encoder =readEncoder(RIGHT_H);
#endif
//  Serial.println("leftPID.Encoder");
//  Serial.println(leftPID.Encoder);
//  Serial.println("rightPID.Encoder");
//  Serial.println(rightPID.Encoder);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
#ifdef L298P    
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
#endif
#ifdef L298P_4WD
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0 || leftPID_h.PrevInput != 0 || rightPID_h.PrevInput != 0) resetPID();
#endif    
    return;
  }

  /* Compute PID update for each motor */
  dorightPID(&rightPID);
  doleftPID(&leftPID);
#ifdef L298P_4WD
  dorightPID_h(&rightPID_h);
  doleftPID_h(&leftPID_h);
#endif  

  /* Set the motor speeds accordingly */
  //Serial.println("leftPID.output");
  //Serial.println(leftPID.output);
  //Serial.println("rightPID.output");
  //Serial.println(rightPID.output);
#ifdef L298P   
  setMotorSpeeds(leftPID.output, rightPID.output);
#endif

#ifdef L298P_4WD
  setMotorSpeeds(leftPID.output,leftPID_h.output, rightPID.output,rightPID_h.output);
#endif

}
long readPidIn(int i) {
  long pidin=0;
  if (i == LEFT){
    pidin = leftPID.PrevInput;
  }else if (i == RIGHT){
    pidin = rightPID.PrevInput;
  }
#ifdef L298P_4WD
  else if (i== RIGHT_H){
    pidin = rightPID_h.PrevInput;
  }else{
    pidin = leftPID_h.PrevInput;
  }
#endif  
  return pidin;
}

long readPidOut(int i) {
  long pidout=0;
  if (i == LEFT){
    pidout = leftPID.output;
  }else if (i == RIGHT){
    pidout = rightPID.output;
  }
#ifdef L298P_4WD
  else if (i == RIGHT_H){
    pidout = rightPID_h.output;
  }else{
    pidout = leftPID_h.output;
  }
#endif   
  return pidout;
}

