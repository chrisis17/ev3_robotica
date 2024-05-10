#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#define WHEEL_NUM   2

#define LEFT        0
#define RIGHT       1

double w_left = 0.0;
double w_right = 0.0;
double L = 0.33;
double R = 0.04;

void right_vel_cb(const std_msgs::Float32::ConstPtr& wr_msg)
{
  w_left = wr_msg->data;
}

void left_vel_cb(const std_msgs::Float32::ConstPtr& wl_msg)
{
  w_right = wl_msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_odom");
  ros::NodeHandle n;
  ros::Subscriber wr_sub = n.subscribe("w_right", 1, right_vel_cb);
  ros::Subscriber wl_sub = n.subscribe("w_left", 1, left_vel_cb);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  double dt = 0.0; 
  ros::Time last_time = ros::Time::now();

  double x = 0.0;     // Initial position in X [meters]
  double y = 0.0;     // Initial position in Y [meters]
  double th = 0.0;    // Initial orientation of the robot [radians] (positive clock rotation)
  double vx = 0.0;    // Initial Linear velocity in X [meters/seconds]
  double vy = 0.0;    // Initial Linear velocity in Y [meters/seconds]
  double vm = 0.0;    // Initial Linear velocity [meters/seconds]
  double wm = 0.0;    // Initial Angular velocity [radians/seconds]
  
  double pos_left = 0.0;
  double pos_right = 0.0;

  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate loop_rate(5);
  while(ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    vm = (w_right + w_left)*R/2.0;    // Lineal velocity of the robot [meters/seconds]
    wm = (w_right - w_left)*R/L;      // Angular velocity of the robot [radians/seconds]

    vx = vm*cos(th + wm*dt/2);       // Linear velocity in X [meters/seconds]
    vy = vm*sin(th + wm*dt/2);       // Linear velocity in Y [meters/seconds]

    x = x + vm*dt*cos(th + wm*dt/2);  // Position in X [meters]
    y = y + vm*dt*sin(th + wm*dt/2);  // Position in Y [meters]
    th = th + wm*dt;                  // Orientation of the robot [radians] (positive clock rotation)

    x = std::round(x*1000.0)/1000.0;
    y = std::round(y*1000.0)/1000.0;
    th = std::round(th*1000.0)/1000.0;

    pos_left = pos_left + w_left*dt;
    pos_right = pos_right + w_right*dt;

    //ROS_INFO("wR: %f and wL: %f", w_right, w_left);
    //ROS_INFO("dt: %f , vm: %f and wm: %f", dt, vm, wm);
    //ROS_INFO("x: %f , y: %f and th: %f", x, y, th);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vm;
    odom.twist.twist.angular.z = wm;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_pub.publish(odom);
    odom_broadcaster.sendTransform(odom_trans);   //send the transform
    
    // Begin - Wheel state

    static char *joint_states_name[] = {(char*)"base_link__left_wheel", (char*)"base_link__right_wheel"};
    sensor_msgs::JointState joint_states;

    joint_states.header.stamp    = current_time;
    joint_states.header.frame_id = "/base_link";
    joint_states.name.resize(2);
    joint_states.position.resize(2);
    joint_states.velocity.resize(2);

    joint_states.name[0] = "base_link__left_wheel";
    joint_states.name[1] = "base_link__right_wheel";
    joint_states.position[0] = pos_left;
    joint_states.position[1] = pos_right;
    joint_states.velocity[0] = w_left;
    joint_states.velocity[1] = w_right;

    js_pub.publish(joint_states);

    // End - Wheel state

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}