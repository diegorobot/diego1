/* *************************************************************
   Encoder definitions
   
   Add a "#if defined" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#if defined ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined ARDUINO_ENC_COUNTER
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
#ifdef L298P_4WD  
  volatile long left_h_enc_pos =0L;
  volatile long right_h_enc_pos =0L;
#endif
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
#ifdef L298P_4WD        
     //uint8_t pindt=PIND;
     static uint8_t enc_last_h=0;
#endif      
     //Serial.print("PIND:");  
     //Serial.println(PIND);     
	   enc_last <<=2; //shift previous state two places
	   enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

#ifdef L298P_4WD
     enc_last_h<<=2;
     enc_last_h |=(PIND & (3 << 4))>>4;
#endif 
  
  	 left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
#ifdef L298P_4WD   
     left_h_enc_pos +=ENC_STATES[(enc_last_h & 0x0f)];
#endif    
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
     static uint8_t enc_last=0;
#ifdef L298P_4WD        
     //uint8_t pinct=PINC;
     static uint8_t enc_last_h=0;
#endif   
//     Serial.println("IS2 PCINT1----------:");       
//     Serial.print("PINC:");  
//     Serial.println(PINC);  
//     Serial.print("before cal enc_last:");  
//     Serial.println(enc_last); 	
	   enc_last <<=2; //shift previous state two places
	   enc_last |= (PINC & (3 << 2)) >> 2; //read the current state into lowest 2 bits
//     Serial.print("enc_last:");  
//     Serial.println(enc_last);

#ifdef L298P_4WD
//     Serial.print("before cal enc_last_h:");  
//     Serial.println(enc_last_h);
     enc_last_h<<=2;
     enc_last_h |=(PINC & 3);
//     Serial.print("enc_last_h:");  
//     Serial.println(enc_last_h);
#endif  
  	 right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//     Serial.print("right_enc_pos:");  
//     Serial.println(right_enc_pos);
#ifdef L298P_4WD   
     right_h_enc_pos +=ENC_STATES[(enc_last_h & 0x0f)];
//     Serial.print("right_h_enc_pos:");  
//     Serial.println(right_h_enc_pos);
#endif
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return 0-left_enc_pos;
    else if(i == RIGHT) return right_enc_pos;
#ifdef L298P_4WD 
    else if (i ==LEFT_H) return 0-left_h_enc_pos;
    else return right_h_enc_pos;
#endif    
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else if (i==RIGHT){ 
      right_enc_pos=0L;
      return;
    }
#ifdef L298P_4WD 
    else if (i==LEFT_H){
      left_h_enc_pos=0L;
      return;
    }else{
      right_h_enc_pos=0L;
      return;
    }
#endif    
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

