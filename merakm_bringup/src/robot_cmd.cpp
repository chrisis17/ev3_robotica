#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

double w_left = 0.0;
double w_right = 0.0;
double L = 0.33;
double R = 0.04;

double vel_x = 0.0;
double ang_z = 0.0;


void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& cmd_msg) {

  vel_x = cmd_msg->linear.x;
  ang_z = cmd_msg->angular.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_cmd");
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, cmd_vel_cb);

  ros::Publisher w_right_pub = nh.advertise<std_msgs::Float32>("w_right", 1);
  ros::Publisher w_left_pub = nh.advertise<std_msgs::Float32>("w_left", 1);

  ros::Rate loop_rate(300);
  while(ros::ok()) {
    
    w_left = -vel_x/R + L*ang_z/(2*R);
    w_right = vel_x/R + L*ang_z/(2*R);

    std_msgs::Float32 cmd_left;
    cmd_left.data = w_left;
    std_msgs::Float32 cmd_right;
    cmd_right.data = w_right;

    w_left_pub.publish(cmd_left);
    w_right_pub.publish(cmd_right);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}