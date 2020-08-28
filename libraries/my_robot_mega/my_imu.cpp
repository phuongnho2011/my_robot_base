#include <Arduino.h>
#include <my_imu.h>

SoftwareWire myWire(A0,A1);

my_imu::my_imu(int addr)
{
    _imu_addr = addr;
}

void my_imu::imu_init(void)
{
    myWire.begin();                      // Initialize comunication
    myWire.beginTransmission(_imu_addr);       // Start communication with MPU6050 // MPU=0x68
    myWire.write(0x6B);                  // Talk to the register 6B
    myWire.write(0x00);                  // Make reset - place a 0 into the 6B register
    myWire.endTransmission(true);        // end the transmission
    delay(20);
}

void my_imu::calculate_IMU_error()
{
    int c = 0;
    while (c < 200) {
    myWire.beginTransmission(_imu_addr);
    myWire.write(0x3B);
    myWire.endTransmission(false);
    myWire.requestFrom(_imu_addr, 6, true);
    _AccX = (myWire.read() << 8 | myWire.read()) / 16384.0 ;
    _AccY = (myWire.read() << 8 | myWire.read()) / 16384.0 ;
    _AccZ = (myWire.read() << 8 | myWire.read()) / 16384.0 ;
    // Sum all readings
    _AccErrorX = _AccErrorX + ((atan((_AccY) / sqrt(pow((_AccX), 2) + pow((_AccZ), 2))) * 180 / PI));
    _AccErrorY = _AccErrorY + ((atan(-1 * (_AccX) / sqrt(pow((_AccY), 2) + pow((_AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  _AccErrorX = _AccErrorX / 200;
  _AccErrorY = _AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    myWire.beginTransmission(_imu_addr);
    myWire.write(0x43);
    myWire.endTransmission(false);
    myWire.requestFrom(_imu_addr, 6, true);
    _GyroX = myWire.read() << 8 | myWire.read();
    _GyroY = myWire.read() << 8 | myWire.read();
    _GyroZ = myWire.read() << 8 | myWire.read();
    // Sum all readings
    _GyroErrorX = _GyroErrorX + (_GyroX / 131.0);
    _GyroErrorY = _GyroErrorY + (_GyroY / 131.0);
    _GyroErrorZ = _GyroErrorZ + (_GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  _GyroErrorX = _GyroErrorX / 200;
  _GyroErrorY = _GyroErrorY / 200;
  _GyroErrorZ = _GyroErrorZ / 200;

  c = 0;
  // Read Vel values 200 times
  while (c < 200) {
    myWire.beginTransmission(_imu_addr);
    myWire.write(0x3B);
    myWire.endTransmission(false);
    myWire.requestFrom(_imu_addr, 6, true);
    _AccX = (myWire.read() << 8 | myWire.read()) / 16384.0 ;
    _AccY = (myWire.read() << 8 | myWire.read()) / 16384.0 ;
    _AccZ = (myWire.read() << 8 | myWire.read()) / 16384.0 ;
    // Sum all readings
    _AccVErrorX = _AccVErrorX + _AccX;
    _AccVErrorY = _AccVErrorY + _AccY;
    c++;
  }
  _AccVErrorX = _AccVErrorX/200;
  _AccVErrorY = _AccVErrorY/200;

  Serial.print("AccErrorX: ");
  Serial.println(_AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(_AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(_GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(_GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(_GyroErrorZ);
  Serial.print("_AccVErrorX: ");
  Serial.println(_AccVErrorX);
  Serial.print("_AccVErrorY: ");
  Serial.println(_AccVErrorY);
}

void my_imu::calculateIMU()
{
    myWire.beginTransmission(_imu_addr);
    myWire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    myWire.endTransmission(false);
    myWire.requestFrom(_imu_addr,14,true);  // request a total of 14 registers 
    
    _AccX = (myWire.read() << 8 | myWire.read()) / 16384.0; // X-axis value
    _AccY = (myWire.read() << 8 | myWire.read()) / 16384.0; // Y-axis value
    _AccZ = (myWire.read() << 8 | myWire.read()) / 16384.0; // Z-axis value
    _Tmp=myWire.read()<<8|myWire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    _GyroX = (myWire.read() << 8 | myWire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    _GyroY = (myWire.read() << 8 | myWire.read()) / 131.0;
    _GyroZ = (myWire.read() << 8 | myWire.read()) / 131.0;
    _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2))) * 180 / PI) - _AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    _accAngleY = (atan(-1 * _AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2))) * 180 / PI) - _AccErrorY; // AccErrorY ~(-1.58)

    _previousTime = _currentTime;        // Previous time is stored before the actual time read
    _currentTime = millis();            // Current time actual time read
    _elapsedTime = (_currentTime - _previousTime) / 1000; // Divide by 1000 to get seconds

    _GyroX = _GyroX - _GyroErrorX; // GyroErrorX ~(-0.56)
    _GyroY = _GyroY - _GyroErrorY; // GyroErrorY ~(2)
    _GyroZ = _GyroZ - _GyroErrorZ; // GyroErrorZ ~ (-0.8)
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    _gyroAngleX = _gyroAngleX + _GyroX * _elapsedTime; // deg/s * s = deg
    _gyroAngleY = _gyroAngleY + _GyroY * _elapsedTime;
    _AccX = _AccX - _AccVErrorX;
    _AccY = _AccY - _AccVErrorY;
    _VelX = _VelX + _AccX*_elapsedTime;
    _VelY = _VelY + _AccY*_elapsedTime;

    _yaw =  _yaw + _GyroZ * _elapsedTime;
    // Complementary filter - combine acceleromter and gyro angle values
    _roll = 0.96 * _gyroAngleX + 0.04 * _accAngleX;
    _pitch = 0.96 * _gyroAngleY + 0.04 * _accAngleY;
}

float my_imu::getVelX(void)
{
    return _VelX;
}
float my_imu::getVelY(void)
{
    return _VelY;
}
float my_imu::getroll(void)
{
    return _roll;   
}
float my_imu::getpitch(void)
{
    return _pitch;  
} 
float my_imu::getyaw(void)
{
    return _yaw;   
}