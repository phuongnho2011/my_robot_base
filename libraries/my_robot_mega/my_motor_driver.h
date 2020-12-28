#ifndef my_motor_driver_h
#define my_motor_driver_h

#include <Arduino.h>
#include <ros.h>

#define INTERRUPT_FL_1 2
#define INTERRUPT_FL_2 39
#define PWM_FL 5
#define I0_FL_1 27
#define I0_FL_2 29

#define INTERRUPT_FR_1 3
#define INTERRUPT_FR_2 41
#define PWM_FR 4
#define I0_FR_1 23
#define I0_FR_2 25

#define INTERRUPT_BL_1 18
#define INTERRUPT_BL_2 43
#define PWM_BL 7
#define I0_BL_1 35
#define I0_BL_2 37

#define INTERRUPT_BR_1 19
#define INTERRUPT_BR_2 45
#define PWM_BR 6
#define I0_BR_1 33
#define I0_BR_2 31

#define SAMPLE_DELAY 1000
#define PULSES_PER_TURN 181.5

#define LIN 0
#define RAD 1

#define LPF_heso 0.08

class motor_driver
{
public:
    motor_driver();
    void init(void);
    void motor_Right(int);
    void motor_Left(int);
    void DemxungFL();
    static void cal_encoderFL();
    void DemxungFR();
    static void cal_encoderFR();
    void DemxungBL();
    static void cal_encoderBL();
    void DemxungBR();
    static void cal_encoderBR();
    int32_t getFLencoder();
    int32_t getFRencoder();
    int32_t getBLencoder();
    int32_t getBRencoder();
    void PID();
    double getSpeedFL();
    double getSpeedFR();
    double getSpeedBL();
    double getSpeedBR();
    void setSetpointFL(float);
    void setSetpointFR(float);
    void setSetpointBL(float);
    void setSetpointBR(float);
    void setpulseFL_PID(float);
    void setpulseFR_PID(float);
    void setpulseBL_PID(float);
    void setpulseBR_PID(float);

private:
    int32_t pulsesFL, pulsesFR;
    int32_t pulsesBL, pulsesBR;
    //PID
    double T;

    double pulseFL_PID;
    double speedFL, setpointFL, pre_speedFL;
    double E1_FL, E1_1_FL, E1_2_FL;
    double alphaFL, betaFL, gamaFL, KpFL, KiFL, KdFL;
    double OutputFL, LastOutputFL;

    double pulseFR_PID;
    double speedFR, setpointFR, pre_speedFR;
    double E1_FR, E1_1_FR, E1_2_FR;
    double alphaFR, betaFR, gamaFR, KpFR, KiFR, KdFR;
    double OutputFR, LastOutputFR;

    double pulseBL_PID;
    double speedBL, setpointBL, pre_speedBL;
    double E1_BL, E1_1_BL, E1_2_BL;
    double alphaBL, betaBL, gamaBL, KpBL, KiBL, KdBL;
    double OutputBL, LastOutputBL;

    double pulseBR_PID;
    double speedBR, setpointBR, pre_speedBR;
    double E1_BR, E1_1_BR, E1_2_BR;
    double alphaBR, betaBR, gamaBR, KpBR, KiBR, KdBR;
    double OutputBR, LastOutputBR;
};

#endif
