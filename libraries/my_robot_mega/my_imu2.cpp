#include <Arduino.h>
#include <my_imu2.h>

SoftwareWire myWire(A2, A3);

my_imu::my_imu(int addr)
{
    _imu_addr = addr;
}

void my_imu::imu_init(void)
{
    myWire.begin();                      // Initialize comunication
    myWire.beginTransmission(_imu_addr); // Start communication with MPU6050 // MPU=0x68
    myWire.write(0x6B);                  // Talk to the register 6B
    myWire.write(0x00);                  // Make reset - place a 0 into the 6B register
    myWire.endTransmission(true);        // end the transmission
    delay(20);
}

void my_imu::calculate_IMU_error()
{
    int c = 0;
    while (c < 500)
    {
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
    myWire.beginTransmission(_imu_addr);
    myWire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    myWire.endTransmission(false);
    myWire.requestFrom(_imu_addr, 14, true); // request a total of 14 registers

    _AccX = (myWire.read() << 8 | myWire.read()) / 16384.0; // X-axis value
    _AccY = (myWire.read() << 8 | myWire.read()) / 16384.0; // Y-axis value
    _AccZ = (myWire.read() << 8 | myWire.read()) / 16384.0; // Z-axis value

    double _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2)))) - accErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    double _accAngleY = (atan(_AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2)))) - accErrorY; // AccErrorY ~(-1.58)
    double _accAngleZ = (atan(_AccZ / sqrt(pow(_AccX, 2) + pow(_AccY, 2)))) - accErrorZ;

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
    _AccX = (myWire.read() << 8 | myWire.read()) / 16384.0; // X-axis value
    _AccY = (myWire.read() << 8 | myWire.read()) / 16384.0; // Y-axis value
    _AccZ = (myWire.read() << 8 | myWire.read()) / 16384.0; // Z-axis value
    _Tmp = myWire.read() << 8 | myWire.read();              // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    _GyroX = (myWire.read() << 8 | myWire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    _GyroY = (myWire.read() << 8 | myWire.read()) / 131.0;
    _GyroZ = (myWire.read() << 8 | myWire.read()) / 131.0;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    double _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2)))) - accErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    double _accAngleY = (atan(_AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2)))) - accErrorY; // AccErrorY ~(-1.58)
    double _accAngleZ = (atan(_AccZ / sqrt(pow(_AccX, 2) + pow(_AccY, 2)))) - accErrorZ;

    double gyroXrate = _GyroX * PI / 180;
    double gyroYrate = _GyroY * PI / 180;
    double gyroZrate = _GyroZ * PI / 180;

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
