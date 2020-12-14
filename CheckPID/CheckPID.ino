#include <my_motor_driver.h>

motor_driver motor;
float value[2] = {0.5,1};

void setup()
{
  motor.init();
  //motor.setSetpointL(100);
  motor.setSetpointR(144);
  Serial.begin(9600);
}

long t = millis();
long t2 = millis();
void loop()
{
  if(millis() - t > 100){
  motor.PID(millis() - t);
  Serial.println(motor.getSpeedR());
  t = millis();
  }
  
  //Serial.println(motor.getRightencoder());
  if(millis() - t2 > 10000)
  {
    motor.setSetpointR(0);
    //motor.setSetpointL(0);
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
