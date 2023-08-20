#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

using namespace std;

ros::Subscriber wheel_velocity_cmdsub;
ros::Publisher  odom_pub;
ros::Timer simulate_timer;

ackermann_msgs::AckermannDrive ackermann_cmd;
double x=0.0;
double y=0.0;
double speed = 0.0;
// double yaw = 0.0;
double yaw = M_PI;

double p_init_x, p_init_y, p_init_z;
double time_resolution = 0.01;
double wheelbase = 2.5;
double max_steer = 0.7854;
double min_speed = -5.555;
double max_speed = 15.278;
bool control_a = false;

bool rcv_cmd = false;

void initParams()
{
	p_init_x = 0.0;
	p_init_y = 0.0;
	p_init_z = 0.0;
}

void rcvVelCmdCallBack(const ackermann_msgs::AckermannDrive cmd)
{	
	               rcv_cmd 	  = true;
	ackermann_cmd    = cmd;
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

	double delta = ackermann_cmd.steering_angle;
	if (delta >= max_steer)
	{
		delta = max_steer;
	}else if (delta<= - max_steer)
	{
		delta = -max_steer;
	}

	if (control_a)
	{
		x = x + speed * cos(yaw) * time_resolution;
		y = y + speed * sin(yaw) * time_resolution;
		yaw = yaw + speed / wheelbase * tan(delta) * time_resolution;
		speed = speed + ackermann_cmd.acceleration * time_resolution;

		if (speed >= max_speed)
		{
			speed = max_speed;
		}else if (speed<= min_speed)
		{
			speed = min_speed;
		}
	}
	else
	{
		speed = ackermann_cmd.speed;
		if (speed >= max_speed)
		{
			speed = max_speed;
		}else if (speed<= min_speed)
		{
			speed = min_speed;
		}
		x = x + speed * cos(yaw) * time_resolution;
		y = y + speed * sin(yaw) * time_resolution;
		yaw = yaw + speed / wheelbase * tan(delta) * time_resolution;
	}

	
	normyaw(yaw);    

	new_odom.pose.pose.position.x  = x;
	new_odom.pose.pose.position.y  = y;
	new_odom.pose.pose.position.z  = 0;
	new_odom.twist.twist.linear.x  = speed * cos(yaw);
	new_odom.twist.twist.linear.y  = speed * sin(yaw);
	new_odom.twist.twist.linear.z  = 0;
	new_odom.twist.twist.angular.z = speed * tan(delta) / wheelbase;	
	
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
  	nh.getParam("simulator/wheelbase", wheelbase);
	nh.getParam("simulator/max_steer", max_steer);
	nh.getParam("simulator/max_speed", max_speed);
	nh.getParam("simulator/min_speed", min_speed);
	nh.getParam("simulator/control_a", control_a);
	  
    wheel_velocity_cmdsub  = nh.subscribe( "cmd", 1000, rcvVelCmdCallBack );
    		     odom_pub  = nh.advertise<nav_msgs::Odometry>("odom", 10);

	ackermann_cmd.acceleration = 0.0;
	ackermann_cmd.steering_angle = 0.0;

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