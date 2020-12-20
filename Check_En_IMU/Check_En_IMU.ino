#include <MPU9250.h>
#include <my_motor_driver.h>

MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  mpu.setup(0x68); 

  delay(5000);

  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  mpu.printCalibration();
}

void loop() {
  if(mpu.update()) {
 // Serial.println(atan2f(mpu.getQuaternionX() * mpu.getQuaternionW() + mpu.getQuaternionY() * mpu.getQuaternionZ(),
 //               0.5f - mpu.getQuaternionZ() * mpu.getQuaternionZ() - mpu.getQuaternionW() * mpu.getQuaternionW()));
    Serial.println(mpu.getYaw());
  }
}
