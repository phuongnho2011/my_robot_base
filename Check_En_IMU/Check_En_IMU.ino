#include <my_imu2.h>
#include <my_motor_driver.h>

my_imu imu(0x68);

void setup() {
  Serial.begin(115200);
  imu.init();
  imu.calculate_IMU_error();
}

void loop() {
  imu.calculateIMU();
  Serial.println(imu.getgyroXrate());
}
