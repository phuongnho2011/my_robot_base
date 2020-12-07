#include <my_motor_driver.h>
motor_driver mt_driver;
double kp, ki, kd, input, output, setpoint;
kp = 5;
ki = 1;
kd = 0.01;
PID PID_TEMP2(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  mt_driver.init();                  
  PID_TEMP2.SetMode(AUTOMATIC);
  PID_TEMP2.SetSampleTime(10);
  PID_TEMP2.SetOutputLimits(-255, 255);
}

void loop() {
  setpoint = 1232;
  input = getLeftencoder();
  myPID_left.Compute();
  mt_driver.motor_Left(output);                
}
void motor_driver::cal_encoderL()
{
  mt_driver.read_EncoderL();
}

void motor_driver::cal_encoderR()
{
  mt_driver.read_EncoderR();
}
