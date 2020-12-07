#include <my_motor_driver.h>

motor_driver::motor_driver()
{
    
}

void motor_driver::init()
{
    pinMode(A1, INPUT_PULLUP); //chan ngat encoder
    pinMode(B1, INPUT_PULLUP); //chan doc encoder
    pinMode(A2, INPUT_PULLUP); //chan ngat encoder
    pinMode(B2, INPUT_PULLUP); //chan doc encoder

    pinMode(EN_R, OUTPUT);   //chan pwm
    pinMode(INT1_R, OUTPUT); //chan DIR1
    pinMode(INT2_R, OUTPUT); //chan DIR2

    pinMode(EN_L, OUTPUT);   //chan pwm
    pinMode(INT1_L, OUTPUT); //chan DIR1
    pinMode(INT2_L, OUTPUT); //chan DIR2
    kp_l = 5;
    ki_l = 1;
    kd_l = 0.01;
    kp_r = 5;
    ki_r = 1;
    kd_r = 0.01;
    PID PID_TEMP(&input, &output, &setpoint, kp_l, ki_l, kd_l, DIRECT);
    myPID_left = PID_TEMP;
    PID_TEMP.SetTunings(kp_r,ki_r,kd_r);
    myPID_right = PID_TEMP;
    
    TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
    myPID_left.SetMode(AUTOMATIC);
    myPID_left.SetSampleTime(10);
    myPID_left.SetOutputLimits(-255, 255);
    myPID_right.SetMode(AUTOMATIC);
    myPID_right.SetSampleTime(10);
    myPID_right.SetOutputLimits(-255, 255);

    attachInterrupt(digitalPinToInterrupt(A1),cal_encoderL,CHANGE);
    attachInterrupt(digitalPinToInterrupt(A2),cal_encoderR,CHANGE);
}

void motor_driver::motor_Right(int Pulse_Width)
{
    if (Pulse_Width > 0)
    {
        analogWrite(EN_R, Pulse_Width);
        digitalWrite(INT1_R, LOW);
        digitalWrite(INT2_R, HIGH);
    }
    else if (Pulse_Width < 0)
    {
        analogWrite(EN_R, abs(Pulse_Width));
        digitalWrite(INT1_R, HIGH);
        digitalWrite(INT2_R, LOW);
    }
    else
    {
        analogWrite(EN_R, Pulse_Width);
        digitalWrite(INT1_R, LOW);
        digitalWrite(INT2_R, LOW);
    }
}

void motor_driver::motor_Left(int Pulse_Width)
{
    if (Pulse_Width > 0)
    {
        analogWrite(EN_L, Pulse_Width);
        digitalWrite(INT1_L, LOW);
        digitalWrite(INT2_L, HIGH);
    }
    else if (Pulse_Width < 0)
    {
        analogWrite(EN_L, abs(Pulse_Width));
        digitalWrite(INT1_L, HIGH);
        digitalWrite(INT2_L, LOW);
    }
    else
    {
        analogWrite(EN_L, Pulse_Width);
        digitalWrite(INT1_L, LOW);
        digitalWrite(INT2_L, LOW);
    }
}

void motor_driver::read_EncoderL()
{
    if ( digitalRead(B1) == 0 ) 
    {
        if ( digitalRead(A1) == 0 ) 
        {
            // A fell, B is low
            pulsesL++; // Moving forward
        } 
        else 
        {
            // A rose, B is high
            pulsesL--; // Moving reverse
        }
    } 
    else 
    {
        if ( digitalRead(A1) == 0 ) 
        {
            pulsesL--; // Moving reverse
        } 
        else 
        {
            // A rose, B is low
            pulsesL++; // Moving forward
        }
    }
}

void motor_driver::read_EncoderR()
{
    if ( digitalRead(B2) == 0 ) 
    {
        if ( digitalRead(A2) == 0 ) 
        {
            // A fell, B is low
            pulsesR--; // Moving forward
        } 
        else
        {
            // A rose, B is high
            pulsesR++; // Moving reverse
        }
    } 
    else 
    {
        if ( digitalRead(A2) == 0 ) 
        {
            pulsesR++; // Moving reverse
        } 
        else 
        {
            // A rose, B is low
            pulsesR--; // Moving forward
        }
    }
}

int32_t motor_driver::getLeftencoder()
{
    return pulsesL;
}

int32_t motor_driver::getRightencoder()
{
    return pulsesR;
}

void motor_driver::control_Motor(const float wheel_rad, const float wheel_sep, float* cmd_value)
{
    setpoint = (cmd_value[LIN] - cmd_value[RAD]*wheel_sep/2)/(2*3.14159265359*wheel_rad)*1232;
    input = getLeftencoder();
    myPID_left.Compute();
    motor_Left(output);
    setpoint = (cmd_value[LIN] + cmd_value[RAD]*wheel_sep/2)/(2*3.14159265359*wheel_rad)*1232;
    input = getRightencoder();
    myPID_right.Compute();
    motor_Right(output);
    //motor_Left((cmd_value[LIN]*0.701/0.22 / wheel_rad)*10 - ((cmd_value[RAD]*4.41/2.75 * wheel_sep) / (2.0 * wheel_rad))*10);
    //motor_Right((cmd_value[LIN]*0.701/0.22 / wheel_rad)*10 + ((cmd_value[RAD]*4.41/2.75 * wheel_sep) / (2.0 * wheel_rad))*10);
}

double motor_driver::getOutput()
{
    return output;
}
