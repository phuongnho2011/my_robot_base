#include <MPU9250.h>
#include <my_motor_driver.h>

MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  mpu.setup(0x68); 

  delay(3000);

  mpu.calibrateAccelGyro();
  Serial.println("CalibrateMag");
  delay(2000);	
  mpu.calibrateMag();

  mpu.printCalibration();
}

void loop() {
  if(mpu.update()) {
 // Serial.println(atan2f(mpu.getQuaternionW() * mpu.getQuaternionZ() + mpu.getQuaternionX() * mpu.getQuaternionY(),
               // 0.5f - mpu.getQuaternionY() * mpu.getQuaternionY() - mpu.getQuaternionZ() * mpu.getQuaternionZ()));
   Serial.println(mpu.getYaw());
  }
}
