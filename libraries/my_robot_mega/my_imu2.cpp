#include <Arduino.h>
#include <my_imu2.h>

// SoftwareWire Wire(A2, A3);
// Them thu de commit

my_imu::my_imu(int addr)
{
    _imu_addr = addr;
}

void my_imu::init(void)
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

    while (c < 200)
    {
        Wire.beginTransmission(_imu_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(_imu_addr, 6, true);               // request a total of 14 registers
        _AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
        _AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
        _AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
        accErrorX = accErrorX + atan(_AccX / sqrt(_AccX * _AccX + _AccZ * _AccZ));
        accErrorY = accErrorY + atan(_AccX / sqrt(_AccY * _AccY + _AccZ * _AccZ));
        c++;
    }
    accErrorX = accErrorX / 200;
    accErrorY = accErrorY / 200;

    c = 0;
    while (c < 200)
    {
        Wire.beginTransmission(_imu_addr);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(_imu_addr, 6, true);
        _GyroX = Wire.read() << 8 | Wire.read();
        _GyroY = Wire.read() << 8 | Wire.read();
        _GyroZ = Wire.read() << 8 | Wire.read();
        // Sum all readings
        GyroErrorX = GyroErrorX + (_GyroX / 131.0);
        GyroErrorY = GyroErrorY + (_GyroY / 131.0);
        GyroErrorZ = GyroErrorZ + (_GyroZ / 131.0);
        c++;
    }
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
}

void my_imu::calculateIMU(void)
{
    Wire.beginTransmission(_imu_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(_imu_addr, 14, true);              // request a total of 14 registers
    _AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    _AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    _AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
    _Tmp = Wire.read() << 8 | Wire.read();              // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    _GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    _GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    _GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    //dt = 0.01;

    _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2)))) - accErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    _accAngleY = (atan(_AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2)))) - accErrorY; // AccErrorY ~(-1.58)
    //_accAngleZ = (atan(_AccZ / sqrt(pow(_AccX, 2) + pow(_AccY, 2)))) - accErrorZ;

    _GyroX = _GyroX - GyroErrorX;
    _GyroY = _GyroY - GyroErrorY;
    _GyroZ = _GyroZ - GyroErrorZ;

    _GyroZ = _GyroZ*PI/180;

    gyroXangle += _GyroX * dt; // Calculate gyro angle without any filter
    gyroYangle += _GyroY * dt;
    gyroZangle += _GyroZ * dt;

    // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * _accAngleX; // Calculate the angle using a Complimentary filter
    // compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * _accAngleY;
    // compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * _accAngleZ;
}

double my_imu::getgyroZangle()
{
    return gyroZangle;
}
double my_imu::getGyroZerror()
{
    return GyroErrorZ;
}

double my_imu::getGyroZ()
{
    return _GyroZ;
}