#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "diablo_sdk/Diablo_Ctrl.h"

using namespace std;

ros::Subscriber diablo_cmd_sub;
ros::Publisher  odom_pub;
ros::Timer simulate_timer;

diablo_sdk::Diablo_Ctrl diablo_cmd;
double x=0.0;
double y=0.0;
double speed = 0.0;
double yaw = M_PI_2;
double yaw_dot = 0.0;

double p_init_x, p_init_y, p_init_z;
double time_resolution = 0.01;
double max_omega = 2.4;
double max_speed = 5.0;

bool rcv_cmd = false;

void initParams()
{
	p_init_x = 0.0;
	p_init_y = 0.0;
	p_init_z = 0.0;
}

void rcvVelCmdCallBack(const diablo_sdk::Diablo_CtrlConstPtr cmd)
{	
	               rcv_cmd 	  = true;
	diablo_cmd    = *cmd;
    // ROS_INFO("in ODOM, the cmd is: a=%f, steer=%f", cmd.drive.acceleration, cmd.drive.steering_angle);
}

void normyaw(double& th)
{
	while (th > M_PI)
		th -= M_PI * 2;
	while (th < -M_PI)
		th += M_PI * 2;
}

void simCallback(const ros::TimerEvent &e)
{
	nav_msgs::Odometry new_odom;

	new_odom.header.stamp       = ros::Time::now();
	new_odom.header.frame_id    = "world";

	speed = diablo_cmd.speed;
	yaw_dot = diablo_cmd.omega;
	if (speed >= max_speed)
	{
		speed = max_speed;
	}else if (speed<= -max_speed)
	{
		speed = -max_speed;
	}
	if (yaw_dot >= max_omega)
	{
		yaw_dot = max_omega;
	}else if (yaw_dot<= -max_omega)
	{
		yaw_dot = -max_omega;
	}
	x = x + speed * cos(yaw) * time_resolution;
	y = y + speed * sin(yaw) * time_resolution;
	yaw = yaw + yaw_dot * time_resolution;
	
	normyaw(yaw);    

	new_odom.pose.pose.position.x  = x;
	new_odom.pose.pose.position.y  = y;
	new_odom.pose.pose.position.z  = 0;
	new_odom.twist.twist.linear.x  = speed * cos(yaw);
	new_odom.twist.twist.linear.y  = speed * sin(yaw);
	new_odom.twist.twist.linear.z  = 0;
	new_odom.twist.twist.angular.z = yaw_dot;	
	
	Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
	Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
	Eigen::Vector3d zC(0, 0, 1);
	Eigen::Matrix3d R2;
	R2.col(0) = xC;
	R2.col(1) = yC;
	R2.col(2) = zC;
	Eigen::Quaterniond q2(R2);
	new_odom.pose.pose.orientation.w = q2.w();
	new_odom.pose.pose.orientation.x = q2.x();
	new_odom.pose.pose.orientation.y = q2.y();
	new_odom.pose.pose.orientation.z = q2.z();

	odom_pub.publish(new_odom);

}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "ugv_kinematic_model_node");
    ros::NodeHandle nh( "~" );

	initParams();
	nh.getParam("simulator/max_omega", max_omega);
	nh.getParam("simulator/max_speed", max_speed);
	  
    diablo_cmd_sub  = nh.subscribe( "command", 1000, rcvVelCmdCallBack );
    odom_pub  = nh.advertise<nav_msgs::Odometry>("odometry", 10);

	diablo_cmd.speed = 0.0;
	diablo_cmd.omega = 0.0;

    simulate_timer = nh.createTimer(ros::Duration(time_resolution), simCallback);
	ros::spin();


    // ros::Rate rate(100);

    // while(ros::ok()) 
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}