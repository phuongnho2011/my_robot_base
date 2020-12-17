#include <my_motor_driver.h>
#include <TimerOne.h>

motor_driver motor;
float value[2] = {0.5,1};

void setup()
{
  motor.init();
  //motor.setSetpointL(100);
  motor.setSetpointR(50);
  Serial.begin(9600);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(PID);
}

void loop()
{ 
  Serial.println(motor.getSpeedR());
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
  motor.PID(20);
}
