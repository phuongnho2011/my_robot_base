#include <my_motor_driver.h>
#include <TimerOne.h>

motor_driver motor;
//

void setup()
{
  motor.init();
  delay(1000);
  motor.setSetpointFR(0); //cong them 5 vong
  motor.setSetpointFL(10); // cong them 3.2
  Serial.begin(9600);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(PID);
}

void loop()
{ 
  Serial.println(motor.getSpeedFL());
  //Serial.println(motor.getFLencoder());
}

void motor_driver::cal_encoderFL()
{
  motor.DemxungFL();
}

void motor_driver::cal_encoderFR()
{
  motor.DemxungFR();
}

void motor_driver::cal_encoderBL()
{
  motor.DemxungBL();
}

void motor_driver::cal_encoderBR()
{
  motor.DemxungBR();
}


void PID()
{
  motor.PID();
}
