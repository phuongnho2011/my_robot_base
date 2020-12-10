#ifndef my_motor_driver_h
#define my_motor_driver_h

#include <Arduino.h>
//#include<TimerOne.h>

#define EN_L 4
#define EN_R 5

#define INT1_L 23
#define INT2_L 25

#define INT1_R 27
#define INT2_R 29

#define A1 2
#define B1 39

#define A2 3
#define B2 41

#define SAMPLE_DELAY 1000
#define PULSES_PER_TURN 181.5

#define LIN 0
#define RAD 1

class motor_driver
{
    public:
        motor_driver();
        void init(void);
        void motor_Right(int);
        void motor_Left(int);
        void control_Motor(const float wheel_rad, const float wheel_sep,float* cmd_value);
        void read_EncoderL();
        static void cal_encoderL();
        void read_EncoderR();
        static void cal_encoderR();
        int32_t getLeftencoder();
        int32_t getRightencoder();
        void PID(double);
	double getOutputL();
    private:
        int32_t pulsesL, pulsesR;
        //PID
	double T;
        double LPF_heso=0.08 ;
        double pulseR_PID;
        double speedR, setpointR, pre_speedR;
        double E1_R, E1_1_R, E1_2_R;
        double alphaR, betaR, gamaR, KpR, KiR, KdR;
        double OutputR, LastOutputR;

        double pulseL_PID;
        double speedL, setpointL, pre_speedL;
        double E1_L, E1_1_L, E1_2_L;
        double alphaL, betaL, gamaL, KpL ,KiL ,KdL;
        double OutputL, LastOutputL;

};

#endif 
