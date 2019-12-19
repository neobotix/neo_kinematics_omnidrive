#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <cstdlib>
#include <chrono>
using namespace std::chrono;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "identification_node");                    //initialize ros node  
  	ros::NodeHandle n1;
	std_msgs::Float64 msg;
	std_msgs::Float64 msg1;
	std_msgs::Float64 msg2;
	high_resolution_clock::time_point t1 = high_resolution_clock::now(); //Start timer
	double secondsPassed;
	double secondsToDelay = 1;
	ros::Publisher velocity = n1.advertise<std_msgs::Float64>("velocity", 1000);
	double vel_1 = 0.05;
	double vel_2 = 0.06;
	double vel_3 = 0.07;

	msg.data = vel_1;
	msg1.data = vel_2;
	msg2.data = vel_3;
	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		duration<double, std::milli> time_count = duration_cast<duration<double, std::milli>>( high_resolution_clock::now() - t1);
		secondsPassed = time_count.count();
		if((int)secondsPassed < 1101)
		{
			velocity.publish(msg);
		}
		if((int)secondsPassed >= 1101 && (int)secondsPassed <1103)
		{

			velocity.publish(msg1);

		}
		if((int)secondsPassed >= 1103)
		{

			velocity.publish(msg2);
		}
		loop_rate.sleep();
	}


}