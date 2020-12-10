#include <my_motor_driver.h>

motor_driver motor;
float value[2] = {0.5,1};

void setup()
{
  motor.init();
  Serial.begin(9600);
}

long t = millis();

void loop()
{
  
  if(millis() - t > 10){
  //motor.PID(millis() - t);
  motor.control_Motor(0.033,0.287,value,millis() - t);
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
