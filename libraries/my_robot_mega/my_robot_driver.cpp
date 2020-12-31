#include <my_motor_driver.h>

motor_driver::motor_driver()
{

}

void motor_driver::init()
{
    pinMode(INTERRUPT_FL_1, INPUT_PULLUP); //chan ngat encoder
    pinMode(INTERRUPT_FL_2, INPUT_PULLUP); //chan doc encoder
    pinMode(INTERRUPT_FR_1, INPUT_PULLUP); //chan ngat encoder
    pinMode(INTERRUPT_FR_2, INPUT_PULLUP); //chan doc encoder
    pinMode(INTERRUPT_BL_1, INPUT_PULLUP); //chan ngat encoder
    pinMode(INTERRUPT_BL_2, INPUT_PULLUP); //chan doc encoder
    pinMode(INTERRUPT_BR_1, INPUT_PULLUP); //chan ngat encoder
    pinMode(INTERRUPT_BR_2, INPUT_PULLUP); //chan doc encoder

    pinMode(PWM_FL, OUTPUT);
    pinMode(I0_FL_1, OUTPUT);
    pinMode(I0_FL_2, OUTPUT);

    pinMode(PWM_FR, OUTPUT);
    pinMode(I0_FR_1, OUTPUT);
    pinMode(I0_FR_2, OUTPUT);

    pinMode(PWM_BL, OUTPUT);
    pinMode(I0_BL_1, OUTPUT);
    pinMode(I0_BL_2, OUTPUT);

    pinMode(PWM_BR, OUTPUT);
    pinMode(I0_BR_1, OUTPUT);
    pinMode(I0_BR_2, OUTPUT);

    T = 0.01;

    speedFL = 0.00, pre_speedFL = 0.00;
    E1_FL = 0, E1_1_FL = 0, E1_2_FL = 0;
    OutputFL = 0, LastOutputFL = 0;
    KpFL = 1000, KiFL = 23.0, KdFL = 15.0;

    speedFR = 0.00, pre_speedFR = 0.00;
    E1_FR = 0, E1_1_FR = 0, E1_2_FR = 0;
    OutputFR = 0, LastOutputFR = 0;
    KpFR = 1000, KiFR = 23.0, KdFR = 15.0;

    speedBL = 0.00, pre_speedBL = 0.00;
    E1_BL = 0, E1_1_BL = 0, E1_2_BL = 0;
    OutputBL = 0, LastOutputBL = 0;
    KpBL = 1000, KiBL = 23.0, KdBL = 15.0;

    speedBR = 0.00, pre_speedBR = 0.00;
    E1_BR = 0, E1_1_BR = 0, E1_2_BR = 0;
    OutputBR = 0, LastOutputBR = 0;
    KpBR = 1000, KiBR = 23.0, KdBR = 15.0;

    attachInterrupt(0, cal_encoderFL, FALLING);
    attachInterrupt(1, cal_encoderFR, FALLING);
    attachInterrupt(5, cal_encoderBL, FALLING);
    attachInterrupt(4, cal_encoderBR, FALLING);
}

void motor_driver::DemxungFL()
{
    if (digitalRead(INTERRUPT_FL_2) == HIGH)
    {
        pulsesFL--;
        pulseFL_PID--;
    }
    else
    {
        pulsesFL++;
        pulseFL_PID++;
    }
}

void motor_driver::DemxungFR()
{
    if (digitalRead(INTERRUPT_FR_2) == HIGH)
    {
        pulsesFR++;
        pulseFR_PID++;
    }
    else
    {
        pulsesFR--;
        pulseFR_PID--;
    }
}

void motor_driver::DemxungBL()
{
    if (digitalRead(INTERRUPT_BL_2) == HIGH)
    {
        pulsesBL--;
        pulseBL_PID--;
    }
    else
    {
        pulsesBL++;
        pulseBL_PID++;
    }
}

void motor_driver::DemxungBR()
{
    if (digitalRead(INTERRUPT_BR_2) == HIGH)
    {
        pulsesBR++;
        pulseBR_PID++;
    }
    else
    {
        pulsesBR--;
        pulseBR_PID--;
    }
}

void motor_driver::PID()
{

    // ---------------FRONT LEFT PID
    speedFL = (pulseFL_PID / 616.0) * (1.0 / T) * 60;
    speedFL = speedFL * LPF_heso + pre_speedFL * (1 - LPF_heso);
    pre_speedFL = speedFL;
    pulseFL_PID = 0;
    E1_FL = setpointFL - speedFL;
    alphaFL = 2 * T * KpFL + KiFL * T * T + 2 * KdFL;
    betaFL = T * T * KiFL - 4 * KdFL - 2 * T * KpFL;
    gamaFL = 2 * KdFL;

    OutputFL = (alphaFL * E1_FL + betaFL * E1_1_FL + gamaFL * E1_2_FL + 2 * T * LastOutputFL / (2 * T));

    LastOutputFL = OutputFL;
    E1_2_FL = E1_1_FL;
    E1_1_FL = E1_FL;

    // ---------------FRONT RIGHT PID

    speedFR = (pulseFR_PID / 616) * (1 / T) * 60;
    speedFR = speedFR * LPF_heso + pre_speedFR * (1 - LPF_heso);
    pre_speedFR = speedFR;
    pulseFR_PID = 0;
    E1_FR = setpointFR - speedFR;
    alphaFR = 2 * T * KpFR + KiFR * T * T + 2 * KdFR;
    betaFR = T * T * KiFR - 4 * KdFR - 2 * T * KpFR;
    gamaFR = 2 * KdFR;

    OutputFR = (alphaFR * E1_FR + betaFR * E1_1_FR + gamaFR * E1_2_FR + 2 * T * LastOutputFR / (2 * T));

    LastOutputFR = OutputFR;
    E1_2_FR = E1_1_FR;
    E1_1_FR = E1_FR;

    // ---------------BACK LEFT PID

    speedBL = (pulseBL_PID / 616) * (1 / T) * 60;
    speedBL = speedBL * LPF_heso + pre_speedBL * (1 - LPF_heso);
    pre_speedBL = speedBL;
    pulseBL_PID = 0;
    E1_BL = setpointBL - speedBL;
    alphaBL = 2 * T * KpBL + KiBL * T * T + 2 * KdBL;
    betaBL = T * T * KiBL - 4 * KdBL - 2 * T * KpBL;
    gamaBL = 2 * KdBL;

    OutputBL = (alphaBL * E1_BL + betaBL * E1_1_BL + gamaBL * E1_2_BL + 2 * T * LastOutputBL / (2 * T));

    LastOutputBL = OutputBL;
    E1_2_BL = E1_1_BL;
    E1_1_BL = E1_BL;

    // ---------------BACK RIGHT PID

    speedBR = (pulseBR_PID / 616) * (1 / T) * 60;
    speedBR = speedBR * LPF_heso + pre_speedBR * (1 - LPF_heso);
    pre_speedBR = speedBR;
    pulseBR_PID = 0;
    E1_BR = setpointBR - speedBR;
    alphaBR = 2 * T * KpBR + KiBR * T * T + 2 * KdBR;
    betaBR = T * T * KiBR - 4 * KdBR - 2 * T * KpBR;
    gamaBR = 2 * KdBR;

    OutputBR = (alphaBR * E1_BR + betaBR * E1_1_BR + gamaBR* E1_2_BR + 2 * T * LastOutputBR / (2 * T));

    LastOutputBR = OutputBR;
    E1_2_BR = E1_1_BR;
    E1_1_BR = E1_BR;

    if (OutputFL > 255)
        OutputFL = 255;
    if (OutputFL < -255)
        OutputFL = -255;

    if (OutputFR > 255)
        OutputFR = 255;
    if (OutputFR < -255)
        OutputFR = -225;

    if (OutputBL > 255)
        OutputBL = 255;
    if (OutputBL < -255)
        OutputBL = -225;
    
    if (OutputBR > 255)
        OutputBR = 255;
    if (OutputBR < -255)
        OutputBR = -225;

    // --- PWM FL
    if (OutputFL > 0)
    {
        analogWrite(PWM_FL, OutputFL);
        digitalWrite(I0_FL_1, LOW);
        digitalWrite(I0_FL_2, HIGH);
    }
    else if (OutputFL < 0)
    {
        analogWrite(PWM_FL, abs(OutputFL));
        digitalWrite(I0_FL_1, HIGH);
        digitalWrite(I0_FL_2, LOW);
    }
    else
    {
        analogWrite(PWM_FL, OutputFL);
        digitalWrite(I0_FL_1, LOW);
        digitalWrite(I0_FL_2, LOW);
    }

    // --- PWM FR
    if (OutputFR > 0)
    {
        analogWrite(PWM_FR, OutputFR);
        digitalWrite(I0_FR_1, LOW);
        digitalWrite(I0_FR_2, HIGH);
    }
    else if (OutputFR < 0)
    {
        analogWrite(PWM_FR, abs(OutputFR));
        digitalWrite(I0_FR_1, HIGH);
        digitalWrite(I0_FR_2, LOW);
    }
    else
    {
        analogWrite(PWM_FR, OutputFR);
        digitalWrite(I0_FR_1, LOW);
        digitalWrite(I0_FR_2, LOW);
    }

    // --- PWM BL
    if (OutputBL > 0)
    {
        analogWrite(PWM_BL, OutputBL);
        digitalWrite(I0_BL_1, LOW);
        digitalWrite(I0_BL_2, HIGH);
    }
    else if (OutputBL < 0)
    {
        analogWrite(PWM_BL, abs(OutputBL));
        digitalWrite(I0_BL_1, HIGH);
        digitalWrite(I0_BL_2, LOW);
    }
    else
    {
        analogWrite(PWM_BL, OutputBL);
        digitalWrite(I0_BL_1, LOW);
        digitalWrite(I0_BL_2, LOW);
    }

    // --- PWM BR
    if (OutputBR > 0)
    {
        analogWrite(PWM_BR, OutputBR);
        digitalWrite(I0_BR_1, LOW);
        digitalWrite(I0_BR_2, HIGH);
    }
    else if (OutputBR < 0)
    {
        analogWrite(PWM_BR, abs(OutputBR));
        digitalWrite(I0_BR_1, HIGH);
        digitalWrite(I0_BR_2, LOW);
    }
    else
    {
        analogWrite(PWM_BR, OutputBR);
        digitalWrite(I0_BR_1, LOW);
        digitalWrite(I0_BR_2, LOW);
    }
}

double motor_driver::getSpeedFL()
{
    return speedFL;
}

double motor_driver::getSpeedFR()
{
    return speedFR;
}

double motor_driver::getSpeedBL()
{
    return speedBL;
}

double motor_driver::getSpeedBR()
{
    return speedBR;
}

void motor_driver::setSetpointFL(float setpoint)
{
    setpointFL = setpoint;
}

void motor_driver::setSetpointFR(float setpoint)
{
    setpointFR = setpoint;
}

void motor_driver::setSetpointBL(float setpoint)
{
    setpointBL = setpoint;
}

void motor_driver::setSetpointBR(float setpoint)
{
    setpointBR = setpoint;
}

void motor_driver::setpulseFL_PID(float pulse)
{
    pulseFL_PID = pulse;
}

void motor_driver::setpulseFR_PID(float pulse)
{
    pulseFR_PID = pulse;
}

void motor_driver::setpulseBL_PID(float pulse)
{
    pulseBL_PID = pulse;
}

void motor_driver::setpulseBR_PID(float pulse)
{
    pulseBR_PID = pulse;
}

int32_t motor_driver::getFLencoder()
{
    return pulsesFL;
}

int32_t motor_driver::getFRencoder()
{
    return pulsesFR;
}

int32_t motor_driver::getBLencoder()
{
    return pulsesBL;
}

int32_t motor_driver::getBRencoder()
{
    return pulsesBR;
}
