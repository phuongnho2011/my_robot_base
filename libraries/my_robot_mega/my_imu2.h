#ifndef my_imu2_h
#define my_imu2_h

#include <Arduino.h>
#include <Wire.h>

class my_imu
{
    public:
        my_imu(int);
        void init(void);
        void calculate_IMU_error(void);
        void calculateIMU(void);
        double getgyroZangle(void);
        double getGyroZerror(void);
        double getGyroZ();
    private:
        /* IMU Data */
        int _imu_addr;
        double _AccX, _AccY, _AccZ;
        double _GyroX, _GyroY, _GyroZ;
        double _accAngleX, _accAngleY, _accAngleZ;
        double  dt;
        int16_t _Tmp;

        double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
        double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter

        double accErrorX, accErrorY, accErrorZ;
        double GyroErrorX, GyroErrorY, GyroErrorZ;
        uint32_t timer;
        uint8_t i2cData[14]; // Buffer for I2C data

};

#endif
