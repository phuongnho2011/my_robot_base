#include <my_motor_driver.h>

// motor_driver mt_driver;
// double kp, ki, kd, input, output, setpoint;
// kp = 5;
// ki = 1;
// kd = 0.01;
// PID PID_TEMP2(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// void setup() {
//   Serial.begin(9600);
//   mt_driver.init();                  
//   PID_TEMP2.SetMode(AUTOMATIC);
//   PID_TEMP2.SetSampleTime(10);
//   PID_TEMP2.SetOutputLimits(-255, 255);
// }

// void loop() {
//   setpoint = 1232;
//   input = getLeftencoder();
//   myPID_left.Compute();
//   mt_driver.motor_Left(output);                
// }
// void motor_driver::cal_encoderL()
// {
//   mt_driver.read_EncoderL();
// }

// void motor_driver::cal_encoderR()
// {
//   mt_driver.read_EncoderR();
// }     

#define PID_FLAG 1
#if PID_FLAG
#endif
#define DC_A 1
#if DC_A
  #define INTERRUPT_A_1 50 
  #define INTERRUPT_A_2 2 
  #define PWM_A 6
  #define I0_A_1 5
  #define I0_A_2 4
  #define INTERRUPT_A 0
#endif

#if DC_A
  float xung_A;
  float tocdo_A = 0;
  float tocdodat_A = 50;
  float pre_tocdo_A = 0;
  
  float E_A = 0;
  float E_A_1 = 0;
  float E_A_2 = 0;
  float alpha_A = 0;
  float beta_A = 0;
  float gama_A = 0;
  float Output_A = 0;
  float LastOutput_A = 0;
#endif



#define DC_B 1
#if DC_B
  #define INTERRUPT_B_1 51 
  #define INTERRUPT_B_2 3
  #define PWM_B 9
  #define I0_B_1 8
  #define I0_B_2 7
  #define INTERRUPT_B 1
#endif

#if DC_A
  float xung_B;
  float tocdo_B = 0;
  float tocdodat_B = 50;
  float pre_tocdo_B = 0;
  
  float E_B = 0;
  float E_B_1 = 0;
  float E_B_2 = 0;
  float alpha_B = 0;
  float beta_B = 0;
  float gama_B = 0;
  float Output_B = 0;
  float LastOutput_B = 0;
#endif


#define SERVO 1
#if SERVO
  #define SERVO_PIN 44
  #define SERVO_ZERO 100
  #define COUNT_CONTROL 10
  int count_control = 0;
  Servo servo; 
  float servo_pos = SERVO_ZERO;
#endif

#define KP_A 880
#define KI_A 10.0
#define KD_A 20.0

#define KP_B 880
#define KI_B 10.0
#define KD_B 20.0


#define LPF_heso 0.08
#define _T 10.0
#define RADIO_MOTOR 330

bool servo_control = false;
void setup() {
  #if DC_A
    pinMode(INTERRUPT_A_1,INPUT_PULLUP);//chan ngat encoder, ngat0
    pinMode(INTERRUPT_A_2,INPUT_PULLUP);//chan doc encoder
    pinMode(PWM_A,OUTPUT);//chan pwm
    pinMode(I0_A_1,OUTPUT);//chan DIR1
    pinMode(I0_A_2,OUTPUT);//chan DIR2
    attachInterrupt(INTERRUPT_A,Demxung_A,FALLING);
  #endif
  #if DC_B
    pinMode(INTERRUPT_B_1,INPUT_PULLUP);//chan ngat encoder, ngat0
    pinMode(INTERRUPT_B_2,INPUT_PULLUP);//chan doc encoder
    pinMode(PWM_B,OUTPUT);//chan pwm
    pinMode(I0_B_1,OUTPUT);//chan DIR1
    pinMode(I0_B_2,OUTPUT);//chan DIR2
    attachInterrupt(INTERRUPT_B,Demxung_B,FALLING);
  #endif

  #if SERVO
    pinMode(SERVO_PIN,OUTPUT);
    servo.attach(SERVO_PIN);
    delay(15);
  #endif
  


  Serial.begin(9600);
  Serial.println("Setup");
}
long time_check = millis();
void loop() {
  if (millis()-time_check>_T){
    PID(millis()-time_check);
    time_check = millis();
  }
  #if DC_A
    //Serial.println(tocdo_A);
  #endif
  #if DC_B
    //Serial.println(tocdo_B);
  #endif

  #if SERVO
    if (count_control>=COUNT_CONTROL){
      servo_control = true;
      servo.write(servo_pos);
      servo_control = false;
      count_control =0;
      
    }
    count_control +=1;
  #endif
}
#if DC_A
  void Demxung_A()
  {
    if(digitalRead(INTERRUPT_A_2)==LOW)
      xung_A++;
    else
      xung_A--;
  }
#endif
#if DC_B
  void Demxung_B()
  {
    if(digitalRead(INTERRUPT_B_2)==LOW)
      xung_B++;
    else
      xung_B--;
  }
#endif
#if PID_FLAG
  void PID(long time)
  {
    
    float T = time/1000.0;
    #if DC_A
      tocdo_A = (xung_A/RADIO_MOTOR)*(1/T)*60;
      tocdo_A = tocdo_A * LPF_heso + pre_tocdo_A*(1-LPF_heso);
      pre_tocdo_A = tocdo_A;
      xung_A=0;
      E_A=tocdodat_A-tocdo_A;
      
      alpha_A= 2*T*KP_A + KI_A*T*T+ 2*KD_A;
      beta_A=T*T*KI_A-4*KD_A-2*T*KP_A;
      gama_A=2*KD_A;
      
      Output_A=(alpha_A*E_A + beta_A*E_A_1 + gama_A*E_A_2 +2*T*LastOutput_A/(2*T));
      
      LastOutput_A=Output_A;
      E_A_2=E_A_1;
      E_A_1=E_A;
      
      if(Output_A > 255)
        Output_A=255;
      if(Output_A < -255)
        Output_A=-255;
    #endif
  
    #if DC_B

      tocdo_B = (xung_B/RADIO_MOTOR)*(1/T)*60;
      tocdo_B = tocdo_B * LPF_heso + pre_tocdo_B*(1-LPF_heso);
      pre_tocdo_B = tocdo_B;
      xung_B=0;
      E_B=tocdodat_B-tocdo_B;
      
      alpha_B= 2*T*KP_B + KI_B*T*T+ 2*KD_B;
      beta_B=T*T*KI_B-4*KD_B-2*T*KP_B;
      gama_B=2*KD_B;
      
      Output_B=(alpha_B*E_B + beta_B*E_B_1 + gama_B*E_B_2 +2*T*LastOutput_B/(2*T));
      
      LastOutput_B=Output_B;
      E_B_2=E_B_1;
      E_B_1=E_B;
      
      if(Output_B > 255)
        Output_B=255;
      if(Output_B < -255)
        Output_B=-255;
    #endif
  
    #if DC_A
      if(Output_A>0)
      {
        analogWrite(PWM_A,Output_A);
        digitalWrite(I0_A_1,LOW);
        digitalWrite(I0_A_2,HIGH);;
      }
      else
      {
        analogWrite(PWM_A,abs(Output_A));
        digitalWrite(I0_A_1,HIGH);
        digitalWrite(I0_A_2,LOW);
      }
    #endif
  
    #if DC_B
      if(Output_B>0)
      {
        analogWrite(PWM_B,Output_B);
        digitalWrite(I0_B_1,LOW);
        digitalWrite(I0_B_2,HIGH);;
      }
      else
      {
        analogWrite(PWM_B,abs(Output_B));
        digitalWrite(I0_B_1,HIGH);
        digitalWrite(I0_B_2,LOW);
      }
    #endif
  }
#endif
