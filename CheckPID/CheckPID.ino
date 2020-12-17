#include <my_motor_driver.h>
#include <TimerOne.h>

motor_driver motor;
float value[2] = {0.5,1};

void setup()
{
  motor.init();
  delay(1000);
  motor.setSetpointR(25); //cong them 5 vong
  motor.setSetpointL(23.2); // cong them 3.2
  Serial.begin(9600);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(PID);
}

void loop()
{ 
  Serial.println(motor.getSpeedL());
  //Serial.println(motor.getLeftencoder());
}

void motor_driver::cal_encoderL()
{
  motor.read_EncoderL();
}

void motor_driver::cal_encoderR()
{
  motor.read_EncoderR();
}

void PID()
{
  motor.PID(10);
}
