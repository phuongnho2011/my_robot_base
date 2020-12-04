#ifndef my_motor_driver_h
#define my_motor_driver_h

#include <Arduino.h>
#include <PID_v1.h>

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

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

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
        double getOutput();
    private:
        double kp_l, ki_l, kd_l, input, output, setpoint; 
        double kp_r, ki_r, kd_r;
        PID myPID_left;
        PID myPID_right;
        int32_t pulsesL, pulsesR;

};

#endif 