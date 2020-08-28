#ifndef my_imu_h
#define my_imu_h

#include <Arduino.h>
#include <SoftwareWire.h>

class my_imu
{
    public:
        my_imu(int);
        void imu_init(void);
        void calculate_IMU_error(void);
        void calculateIMU(void);
        float getVelX(void);
        float getVelY(void);
        float getroll(void);
        float getpitch(void); 
        float getyaw(void);
    private:
        int _imu_addr, _Tmp;
        float _AccX, _AccY, _AccZ;
        float _GyroX, _GyroY, _GyroZ;
        float _accAngleX, _accAngleY, _gyroAngleX, _gyroAngleY, _gyroAngleZ, _VelX, _VelY;
        float _roll, _pitch, _yaw;
        float _AccErrorX, _AccErrorY, _GyroErrorX, _GyroErrorY, _GyroErrorZ, _AccVErrorX, _AccVErrorY;
        float _elapsedTime, _currentTime, _previousTime;
};

#endif
