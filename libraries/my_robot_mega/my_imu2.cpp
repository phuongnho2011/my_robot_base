#include <Arduino.h>
#include <my_imu2.h>

// SoftwareWire Wire(A2, A3);

my_imu::my_imu(int addr)
{
    _imu_addr = addr;
}

void my_imu::imu_init(void)
{
    Wire.begin();                      // Initialize comunication
    Wire.beginTransmission(_imu_addr); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                  // Talk to the register 6B
    Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);        // end the transmission
    delay(20);
}

void my_imu::calculate_IMU_error()
{
    int c = 0;
   
    while (c < 500)
    {
        Wire.beginTransmission(_imu_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(_imu_addr, 6, true); // request a total of 14 registers
        _AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
        _AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
        _AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
        accErrorX = accErrorX + atan(_AccX / sqrt(_AccX * _AccX + _AccZ * _AccZ));
        accErrorY = accErrorY + atan(_AccX / sqrt(_AccY * _AccY + _AccZ * _AccZ));
        accErrorZ = accErrorZ + atan(_AccZ / sqrt(_AccX * _AccX + _AccY * _AccY));
        c++;
    }
    accErrorX = accErrorX / 500;
    accErrorY = accErrorY / 500;
    accErrorZ = accErrorZ / 500;
}

void my_imu::setupoffsetIMU()
{
    Wire.beginTransmission(_imu_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(_imu_addr, 6, true); // request a total of 14 registers

    _AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    _AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    _AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2)))) - accErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    _accAngleY = (atan(_AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2)))) - accErrorY; // AccErrorY ~(-1.58)
    _accAngleZ = (atan(_AccZ / sqrt(pow(_AccX, 2) + pow(_AccY, 2)))) - accErrorZ;

    gyroXangle = _accAngleX;
    gyroYangle = _accAngleY;
    gyroZangle = _accAngleZ;

    compAngleX = _accAngleX;
    compAngleY = _accAngleY;
    compAngleZ = _accAngleZ;
    // Complementary filter - combine acceleromter and gyro angle values
    timer = micros();
}

void my_imu::calculateIMU(void)
{
    Wire.beginTransmission(_imu_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(_imu_addr, 14, true); // request a total of 14 registers
    _AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    _AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    _AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
    _Tmp = Wire.read() << 8 | Wire.read();              // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    _GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    _GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    _GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2)))) - accErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    _accAngleY = (atan(_AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2)))) - accErrorY; // AccErrorY ~(-1.58)
    _accAngleZ = (atan(_AccZ / sqrt(pow(_AccX, 2) + pow(_AccY, 2)))) - accErrorZ;

    gyroXrate = _GyroX * PI / 180;
    gyroYrate = _GyroY * PI / 180;
    gyroZrate = _GyroZ * PI / 180;

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * _accAngleX; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * _accAngleY;
    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * _accAngleZ;
}

double my_imu::getcompAngleX()
{
    return compAngleX;
}
double my_imu::getcompAngleY()
{
    return compAngleY;
}
double my_imu::getcompAngleZ()
{
    return compAngleZ;
}
