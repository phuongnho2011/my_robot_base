#include <my_imu2.h>
#include <my_motor_driver.h>

my_imu mpu6050(0x68);

void setup() {
  // put your setup code here, to run once:
  mpu6050.init();
  mpu6050.calculate_IMU_error();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.calculateIMU();
  Serial.println(mpu6050.getcompAngleX());
}
