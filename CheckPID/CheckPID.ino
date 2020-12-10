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
  
  if(millis() - t > 10){
  motor.PID(millis() - t);
  //Serial.println(motor.getRightencoder());
  Serial.println(motor.getOutputL());
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
