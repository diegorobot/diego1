/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC2  //pin A2
  #define RIGHT_ENC_PIN_B PC3   //pin A3

#ifdef L298P_4WD
  #define LEFT_H_ENC_PIN_A PD4  //pin 4
  #define LEFT_H_ENC_PIN_B PD5  //pin 5
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_H_ENC_PIN_A PC0  //pin A0
  #define RIGHT_H_ENC_PIN_B PC1   //pin A1
#endif  
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

