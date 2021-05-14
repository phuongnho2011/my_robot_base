#include <my_robot_core_config.h>

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

unsigned int lastTime;
float rpm;
volatile unsigned int pulseL ,pulseR;

char log_msg[50];
char result[8];

unsigned long previousTime = 0;
unsigned long Time;
float velocity;

double w_r = 0, w_l = 0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0275, wheel_sep = 0.360;
ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50; 
double speed_ang = 0, speed_lin = 0;


//Motor_initalization
void Motor_init();

//Control_Motor
void Motor_right(int Pulse_Width);
void Motor_left(int Pulse_Width);
void Count_Left();
void Count_Right();
void Calculate_Velocity();

void messageCb( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup() {
  Motor_init();
  attachInterrupt(digitalPinToInterrupt(A1),Count_Left,FALLING);
  Serial.begin(9600);
}

void loop() {


}

void Motor_init()
{
  pinMode(A1,INPUT_PULLUP);//chan ngat encoder
  pinMode(B1,INPUT_PULLUP);//chan doc encoder
  pinMode(A2,INPUT_PULLUP);//chan ngat encoder
  pinMode(B2,INPUT_PULLUP);//chan doc encoder

  pinMode(EN_R,OUTPUT);//chan pwm
  pinMode(INT1_R,OUTPUT);//chan DIR1
  pinMode(INT2_R,OUTPUT);//chan DIR2

  pinMode(EN_L,OUTPUT);//chan pwm
  pinMode(INT1_L,OUTPUT);//chan DIR1
  pinMode(INT2_L,OUTPUT);//chan DIR2
}

void Motor_right(int Pulse_Width)
{
  if (Pulse_Width > 0)
  {
    analogWrite(EN_R,Pulse_Width);
    digitalWrite(INT1_R, LOW);
    digitalWrite(INT2_R,HIGH);
  }
  else if (Pulse_Width < 0)
  {
    analogWrite(EN_R,abs(Pulse_Width));
    digitalWrite(INT1_R,HIGH);
    digitalWrite(INT2_R,LOW); 
  }
  else
  {
    analogWrite(EN_R,Pulse_Width);
    digitalWrite(INT1_R,LOW);
    digitalWrite(INT2_R,LOW);
  }
}

void Motor_left(int Pulse_Width)
{
  if (Pulse_Width > 0)
  {
    analogWrite(EN_L,Pulse_Width);
    digitalWrite(INT1_L, LOW);
    digitalWrite(INT2_L,HIGH);
  }
  else if (Pulse_Width < 0)
  {
    analogWrite(EN_L,abs(Pulse_Width));
    digitalWrite(INT1_L,HIGH);
    digitalWrite(INT2_L,LOW); 
  }
  else
  {
    analogWrite(EN_L,Pulse_Width);
    digitalWrite(INT1_L,LOW);
    digitalWrite(INT2_L,LOW);
  }
}

void Count_Left()
{
     ++pulseL;
     Serial.print(pulseL);
}
