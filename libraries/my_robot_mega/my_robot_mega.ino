#include <my_imu.h>
#include <my_robot_core_config.h>

my_imu mpu(0x68);

void setup() {
  Serial.begin(19200);
  mpu.imu_init();
  mpu.calculate_IMU_error();
}

void loop() {
  mpu.calculateIMU();
  Serial.print(mpu.getVelX());
  Serial.print("/");
  Serial.print(mpu.getVelY());
  Serial.print("/");
  Serial.println(mpu.getyaw());
}
