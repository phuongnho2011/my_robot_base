#ifndef my_motor_driver_h
#define my_motor_driver_h

#include <Arduino.h>

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
        void cal_Velocity(void);
        void control_Motor(const float wheel_rad, const float wheel_sep,float* cmd_value);
        void read_EncoderL();
        static void cal_encoderL();
        void read_EncoderR();
        static void cal_encoderR();
        int32_t getLeftencoder();
        int32_t getRightencoder();
    private:
        unsigned int lastTime;
        float rpm;
        int32_t pulsesL, pulsesR;
        unsigned long previousTime = 0;
        unsigned long Time;   
        float velocity;
};

#endif 