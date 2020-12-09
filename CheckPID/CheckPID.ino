#include <my_motor_driver.h>

motor_driver motor;

void setup()
{
  motor.init();
  Serial.begin(9600);
}

long t = millis();

void loop()
{
  
  if(millis() - t > 100){
//  Serial.println(millis()-t);
  motor.PID(millis() - t - 1);
  t = millis();
  }
}

void motor_driver::cal_encoderL()
{
  motor.read_EncoderL();
}

void motor_driver::cal_encoderR()
{
  motor.read_EncoderR();
}
