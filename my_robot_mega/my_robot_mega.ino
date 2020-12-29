#include <Wire.h>
#include <MPU6050.h>
#include <my_motor_driver.h>
#include <my_robot_core_config.h>
#include <TimerOne.h>

void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  tf_broadcaster.init(nh);

  // setting for motors
  mt_driver.init();

  //setting for imu
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    ;

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

  // setting for slam and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  Timer1.initialize(10000);
  Timer1.attachInterrupt(PID);

  prev_update_time = millis();
}

char test[50];
unsigned int t, temp;
void loop()
{
  delayMicroseconds(300);
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    updateMotorInfo(mt_driver.getFLencoder(), mt_driver.getFRencoder(), mt_driver.getBLencoder(), mt_driver.getBRencoder());
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    if (t - tTime[6] > CONTROL_MOTOR_TIMEOUT)
    {
      mt_driver.setSetpointFL(0);
      mt_driver.setSetpointFR(0);
      mt_driver.setSetpointBL(0);
      mt_driver.setSetpointBR(0);
      mt_driver.setpulseFL_PID(0);
      mt_driver.setpulseFR_PID(0);
      mt_driver.setpulseBL_PID(0);
      mt_driver.setpulseBR_PID(0);
    }
    else
    {
      //mt_driver.setSetpointL((goal_velocity_from_cmd[LINEAR] - goal_velocity_from_cmd[ANGULAR] * WHEEL_SEPRATION / 2) / (2 * 3.14159265359 * WHEEL_RADIUS) * 60);
      //mt_driver.setSetpointR((goal_velocity_from_cmd[LINEAR] + goal_velocity_from_cmd[ANGULAR] * WHEEL_SEPRATION / 2) / (2 * 3.14159265359 * WHEEL_RADIUS) * 60);
      mt_driver.setSetpointFL((1 / WHEEL_RADIUS) * (goal_velocity_from_cmd[LINEARX] - goal_velocity_from_cmd[LINEARY] - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * goal_velocity_from_cmd[ANGULAR]) * radtorpm); //Tar_wheel_front_left값 계산 수식
      mt_driver.setSetpointFR((1 / WHEEL_RADIUS) * (goal_velocity_from_cmd[LINEARX] + goal_velocity_from_cmd[LINEARY] + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * goal_velocity_from_cmd[ANGULAR]) * radtorpm); //Tar_wheel_front_right값 계산 수식
      mt_driver.setSetpointBL((1 / WHEEL_RADIUS) * (goal_velocity_from_cmd[LINEARX] + goal_velocity_from_cmd[LINEARY] - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * goal_velocity_from_cmd[ANGULAR]) * radtorpm); //Tar_wheel_rear_left값 계산 수식
      mt_driver.setSetpointBR((1 / WHEEL_RADIUS) * (goal_velocity_from_cmd[LINEARX] - goal_velocity_from_cmd[LINEARY] + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * goal_velocity_from_cmd[ANGULAR]) * radtorpm); //Tar_wheel_rear_right값 계산 수식
    }
    tTime[0] = t;
  }

  nh.spinOnce();
  waitForSerialLink(nh.connected());
}

void PID()
{
  mt_driver.PID();
}

void initJointStates(void)
{
  static char *joint_states_name[] = {(char *)"wheel_fleft_joint", (char *)"wheel_fright_joint", (char *)"wheel_bleft_joint", (char *)"wheel_bright_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name = joint_states_name;

  joint_states.name_length = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length = WHEEL_NUM;
}

/*******************************************************************************
  Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index] = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.linear.y = odom_vel[1];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateTF(geometry_msgs::TransformStamped &odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  // char log_msg2[50];

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // sprintf(log_msg2, "Setup TF on Odometry [%i]", int(step_time));
  // nh.loginfo(log_msg2);

  calcOdometry((double)step_time * 0.001);

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
  Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");

        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(joint_state_header_frame_id, "/base_link");
      }

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
  Update variable (initialization)
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

void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};

  joint_states_pos[FLEFT] = PULSE2RAD * (double)mt_driver.getFLencoder();
  joint_states_pos[FRIGHT] = PULSE2RAD * (double)mt_driver.getFRencoder();
  joint_states_pos[BLEFT] = PULSE2RAD * (double)mt_driver.getBLencoder();
  joint_states_pos[BRIGHT] = PULSE2RAD * (double)mt_driver.getBRencoder();

  joint_states_vel[FLEFT] = last_velocity[FLEFT];
  joint_states_vel[FRIGHT] = last_velocity[FRIGHT];
  joint_states_vel[BLEFT] = last_velocity[BLEFT];
  joint_states_vel[BRIGHT] = last_velocity[BRIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

void updateMotorInfo(int32_t fleft_pulse, int32_t fright_pulse, int32_t bleft_pulse, int32_t bright_pulse)
{
  //int32_t current_pulse = 0;
  static int32_t last_pulse[WHEEL_NUM] = {0, 0, 0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_pulse[index] = 0;
      last_pulse[index] = 0;
      last_rad[index] = 0.0;
      last_velocity[index] = 0.0;
    }

    last_pulse[FLEFT] = fleft_pulse;
    last_pulse[FRIGHT] = fright_pulse;
    last_pulse[BLEFT] = bleft_pulse;
    last_pulse[BRIGHT] = bright_pulse;
    init_encoder = false;
    return;
  }

  last_diff_pulse[FLEFT] = fleft_pulse - last_pulse[FLEFT];
  last_pulse[FLEFT] = fleft_pulse;
  last_rad[FLEFT] += PULSE2RAD * (double)last_diff_pulse[FLEFT];

  last_diff_pulse[FRIGHT] = fright_pulse - last_pulse[FRIGHT];
  last_pulse[FRIGHT] = fright_pulse;
  last_rad[FRIGHT] += PULSE2RAD * (double)last_diff_pulse[FRIGHT];

  last_diff_pulse[BLEFT] = bleft_pulse - last_pulse[BLEFT];
  last_pulse[BLEFT] = bleft_pulse;
  last_rad[BLEFT] += PULSE2RAD * (double)last_diff_pulse[BLEFT];

  last_diff_pulse[BRIGHT] = bright_pulse - last_pulse[BRIGHT];
  last_pulse[BRIGHT] = bright_pulse;
  last_rad[BRIGHT] += PULSE2RAD * (double)last_diff_pulse[BRIGHT];
}

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEARX] = constrain(cmd_vel_msg.linear.x, MIN_LINEAR_VELOCITYX, MAX_LINEAR_VELOCITYX);
  goal_velocity_from_cmd[LINEARY] = constrain(cmd_vel_msg.linear.y, MIN_LINEAR_VELOCITYY, MAX_LINEAR_VELOCITYY);
  goal_velocity_from_cmd[ANGULAR] = constrain(cmd_vel_msg.angular.z, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  if (cmd_vel_msg.linear.x == 0 && cmd_vel_msg.angular.z == 0 && cmd_vel_msg.linear.y == 0)
  {
    mt_driver.setpulseFL_PID(0);
    mt_driver.setpulseFR_PID(0);
    mt_driver.setpulseBL_PID(0);
    mt_driver.setpulseBR_PID(0);
  }

  tTime[6] = millis();
}

bool calcOdometry(double diff_time)
{
  float yaw = 0;
  Vector norm;
  double wheel_fl, wheel_fr, wheel_bl, wheel_br; // rotation value of wheel [rad]
  double delta_x, delta_y, theta, delta_theta;
  static double last_theta = 0.0;
  double vx, vy, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_fl = wheel_fr = wheel_bl = wheel_br = 0.0;
  delta_x = delta_y = delta_theta = theta = 0.0;
  vx = vy = w = 0.0;
  step_time = 0;
  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_fl = PULSE2RAD * (double)last_diff_pulse[FLEFT];
  wheel_fr = PULSE2RAD * (double)last_diff_pulse[FRIGHT];
  wheel_bl = PULSE2RAD * (double)last_diff_pulse[FLEFT];
  wheel_br = PULSE2RAD * (double)last_diff_pulse[FRIGHT];

  if (isnan(wheel_fl))
    wheel_fl = 0.0;

  if (isnan(wheel_fr))
    wheel_fr = 0.0;

  if (isnan(wheel_bl))
    wheel_bl = 0.0;

  if (isnan(wheel_br))
    wheel_br = 0.0;

  norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * step_time;
  theta = yaw * PI / 180;
  delta_theta = theta - last_theta;

  w = delta_theta / step_time;
  //vx = goal_velocity_from_cmd[LINEARX];
  //vy = goal_velocity_from_cmd[ANGULAR];
  vx = (mt_driver.getSpeedFL() + mt_driver.getSpeedFR() + mt_driver.getSpeedBL() + mt_driver.getSpeedBR())*(WHEEL_RADIUS/4);
  vy = (- mt_driver.getSpeedFL() + mt_driver.getSpeedFR() + mt_driver.getSpeedBL() - mt_driver.getSpeedBR())*(WHEEL_RADIUS/4);

  delta_x = (vx * cos(w) - vy * sin(w)) * step_time;
  delta_y = (vx * sin(w) + vy * cos(w)) * step_time;
  
  odom_pose[0] += delta_x;
  odom_pose[1] += delta_y;
  odom_pose[2] += delta_theta;

  odom_vel[0] = vx;
  odom_vel[1] = vy;
  odom_vel[2] = w;

  last_velocity[FLEFT] = wheel_fl / step_time;
  last_velocity[FRIGHT] = wheel_fr / step_time;
  last_velocity[BLEFT] = wheel_bl / step_time;
  last_velocity[BRIGHT] = wheel_br / step_time;
  last_theta = theta;

  return true;
}

ros::Time rosNow()
{
  return nh.now();
}

void motor_driver::cal_encoderFL()
{
  mt_driver.DemxungFL();
}

void motor_driver::cal_encoderFR()
{
  mt_driver.DemxungFR();
}

void motor_driver::cal_encoderBL()
{
  mt_driver.DemxungBL();
}

void motor_driver::cal_encoderBR()
{
  mt_driver.DemxungBR();
}

/*******************************************************************************
  Wait for Serial Link
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
