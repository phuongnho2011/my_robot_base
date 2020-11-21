#ifndef my_robot_core_config_h
#define my_robot_core_config_h

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/* Time update index */
#define CONTROL_MOTOR_TIME_INDEX                0       /*!< Time index control motor */
#define CMD_VEL_PUBLISH_TIME_INDEX              1       /*!< Time index publish velocity */
#define DRIVE_INFORMATION_PUBLISH_TIME_INDEX    2       /*!< Time index publish drive information */
#define IMU_PUBLISH_TIME_INDEX                  3       /*!< Time index publish IMU information */
#define LOG_PUBLISH_TIME_INDEX                  5
#define CONTROL_MOTOR_TIMEOUT_TIME_INDEX        6       /*!< Time index control motor timeout */

/* Frequency of publish/subscribe */
#define CONTROL_MOTOR_SPEED_FREQUENCY          10       /*!< Frequency in Hz to control motor */
#define CONTROL_MOTOR_TIMEOUT                  500      /*!< Period in ms to check control motor timeout */
#define IMU_PUBLISH_FREQUENCY                  15      	/*!< Frequency in Hz to publish IMU information */
#define CMD_VEL_PUBLISH_FREQUENCY              5       	/*!< Frequency in Hz to publish robot velocity */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    5       	/*!< Frequency in Hz to publish drive information */
#define DEBUG_LOG_FREQUENCY                    10       /*!< Frequency in Hz to send log debug messages */

/* Linear & Angular velocity index */
#define WHEEL_NUM       2                       /*!< Num wheel */

#define LEFT            0                       /*!< Left wheel index */
#define RIGHT           1                       /*!< Right wheel index */

#define LINEAR          0                       /*!< Linear velocity index */
#define ANGULAR         1                       /*!< Angular velocity index */

#define PULSE2RAD                         0.005099988  // 1[pulse] * 3.14159265359 / 1232 = 0.001533981f

#define MIN_LINEAR_VELOCITY  -7
#define MAX_LINEAR_VELOCITY  7

#define MIN_ANGULAR_VELOCITY -5
#define MAX_ANGULAR_VELOCITY 5

#define WHEEL_RADIUS 0.0275 //meter
#define WHEEL_SEPRATION 0.318 //meter

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];
char imu_frame_id[30];
char joint_state_header_frame_id[30];

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

// void resetCallback(const std_msgs::Empty& reset_msg);
// void publishCmdVelFromAMEGA(void);

ros::Time rosNow(void);

void updateVariable(bool isConnected);
void updateMotorInfo(int32_t left_pulse, int32_t right_pulse);
void updateTime(void);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void publishDriveInformation(void);
// void updateGyroCali(bool isConnected);
void updateGoalVelocity(void);
// void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

// void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;



// // Command velocity of Turtlebot3 using RC100 remote controller
// geometry_msgs::Twist cmd_vel_amega_msg;
// ros::Publisher cmd_vel_amega_pub("cmd_vel_amega", &cmd_vel_amega_msg);

// Odometry of my_robot
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint(Dynamixel) state of my_robot
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

// /*******************************************************************************
// * SoftwareTimer of Turtlebot3
// *******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_pulse[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];



/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

my_imu imu(0x68);

motor_driver mt_driver;

#endif
