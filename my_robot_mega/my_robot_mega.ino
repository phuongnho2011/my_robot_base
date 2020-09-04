#include <my_imu2.h>
#include <my_robot_core_config.h>

#define EN_R 5
#define EN_L 6

#define INT1_R 7
#define INT2_R 11

#define INT1_L 12
#define INT2_L 13

char log_msg[50];

my_imu imu(0x68);

double w_r = 0, w_l = 0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0275, wheel_sep = 0.360;
int lowSpeed = 200;
int highSpeed = 50; 
double speed_ang = 0, speed_lin = 0;

//Motor_initalization
void Motor_init();

//Control_Motor
void Motor_right(int Pulse_Width);
void Motor_left(int Pulse_Width);

void messageCb( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup() {
  Motor_init();
  nh.initNode();
  nh.subscribe(sub);
  imu.imu_init();
  imu.setupoffsetIMU();
  imu.calculate_IMU_error();
}

void loop() {
  Motor_right(w_r*10);
  Motor_left(w_l*10);
  imu.calculateIMU();
  sprintf(log_msg, "compAngleX [%f]", imu.getcompAngleX());
  nh.loginfo(log_msg);
  sprintf(log_msg, "compAngleY [%f]", imu.getcompAngleY());
  nh.loginfo(log_msg);
  sprintf(log_msg, "compAngleZ [%f]", imu.getcompAngleZ());
  nh.loginfo(log_msg);
  nh.spinOnce();
}

void Motor_init()
{
  pinMode(2,INPUT_PULLUP);//chan ngat encoder
  pinMode(4,INPUT_PULLUP);//chan doc encoder

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
