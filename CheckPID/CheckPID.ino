#include <my_motor_driver.h>
motor_driver mt_driver;

float value[2] = {0.5, 1};
void setup() {
  Serial.begin(9600);
  mt_driver.init();
}

void loop() {
  mt_driver.motor_Right(0);                
}
void motor_driver::cal_encoderL()
{
  mt_driver.read_EncoderL();
}

void motor_driver::cal_encoderR()
{
  mt_driver.read_EncoderR();
}
