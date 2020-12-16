#include <my_imu2.h>
#include <my_motor_driver.h>
#include <my_robot_core_config.h>

void setup() {
  // Initialize ROS node handle, advertise and subscribe the topics 
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  tf_broadcaster.init(nh);

  // setting for imu
  imu.init();

  // setting for motors
  mt_driver.init();

  // setting for slam and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();
}

char test[50];
void loop() {
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());
  
  if((millis() - tTime[0]) >= 1000/CONTROL_MOTOR_SPEED_FREQUENCY)
  {
    if(millis() - tTime[6] >= CONTROL_MOTOR_TIMEOUT)
    {
      mt_driver.setSetpointL(0);
      mt_driver.setSetpointR(0);
      mt_driver.setpulseL_PID(0);
      mt_driver.setpulseR_PID(0);
      tTime[6] = millis();
    }
    else
    {
      mt_driver.PID(millis() - tTime[0]);
      calcOdometry(millis() - tTime[0]);
    }
    tTime[0] = millis();
  }
  
  if ((millis()-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishDriveInformation();  
    tTime[2] = millis();  
  }
  nh.spinOnce();
  waitForSerialLink(nh.connected());
}

void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void publishDriveInformation(void)
{
  ros::Time stamp_now = rosNow();
  
  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);  
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      sprintf(odom_header_frame_id, "odom");
      sprintf(odom_child_frame_id, "base_footprint");  
      sprintf(joint_state_header_frame_id, "base_link");

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      initOdom();
      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  if(cmd_vel_msg.linear.x == 0 && cmd_vel_msg.angular.z == 0)
  {
    mt_driver.setSetpointL(0);
    mt_driver.setSetpointR(0);
    mt_driver.setpulseL_PID(0);
    mt_driver.setpulseR_PID(0);
  }
  mt_driver.setSetpointL((cmd_vel_msg.linear.x + cmd_vel_msg.angular.z*WHEEL_SEPRATION/2)/(2*3.14159265359*WHEEL_RADIUS)*60);
  mt_driver.setSetpointR((cmd_vel_msg.linear.x - cmd_vel_msg.angular.z*WHEEL_SEPRATION/2)/(2*3.14159265359*WHEEL_RADIUS)*60);
  tTime[6] = millis();
}

bool calcOdometry(double diff_time)
{       
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;
  
  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;
  
  wheel_l = PULSE2RADL*mt_driver.getLeftencoder();
  wheel_r = PULSE2RADR*mt_driver.getRightencoder();

  step_time = diff_time;
  
  if(isnan(wheel_l))
    wheel_l = 0.0;

  if(isnan(wheel_r))
    wheel_r = 0.0;

  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  theta = imu.getcompAngleX();
  delta_theta = theta - last_theta;

    // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = (mt_driver.getSpeedL()*PULSE2RADL + mt_driver.getSpeedR()*PULSE2RADR)*WHEEL_RADIUS*1000.0/60.0/2.0;
  w = delta_theta / step_time ;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = mt_driver.getSpeedL()*PULSE2RADL*1000.0/60.0;
  last_velocity[RIGHT] = mt_driver.getSpeedR()*PULSE2RADR*1000.0/60.0;
  last_theta = theta;

  return true;
}

ros::Time rosNow()
{
  return nh.now();
}

void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = PULSE2RADL * (double)mt_driver.getLeftencoder();
  joint_states_pos[RIGHT] = PULSE2RADR * (double)mt_driver.getRightencoder();

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

void motor_driver::cal_encoderL()
{
  mt_driver.read_EncoderL();
}

void motor_driver::cal_encoderR()
{
  mt_driver.read_EncoderR();
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}
