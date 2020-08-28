#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle nh;

nav_msgs::Odometry odom_msg;
ros::Publisher odom("odom", &odom_msg);

unsigned long timer;


void setup()
{
  nh.initNode();
  nh.advertise(odom);
}

void loop()
{
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.pose.pose.orientation.w = 0.0;

  odom_msg.twist.twist.linear.x  = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;
  unsigned long currentMillis = millis();
  if(currentMillis - timer >= 50)
  {
    timer = currentMillis + 50;
    odom.publish(&odom_msg);
  }
  nh.spinOnce();
  delay(1000);
}

