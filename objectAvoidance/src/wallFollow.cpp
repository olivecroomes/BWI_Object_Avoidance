#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <iostream>
bool safeToMove = false;
bool followWall = false;
ros::Publisher velocity_pub;
//the robot will keep running as long as there are


//Point of interest - check type of parameter
//Hope this works!
void unsafeCallback(const geometry_msgs::Twist& msg){
	
	geometry_msgs::Twist current;
	current.linear.x = 0;
		current.angular.z  = 0;
	
	if(safeToMove)
	{
		std::cout << "TELEOP IS MOVING\n" << std::endl;
		current.linear.x= 0.6;
		velocity_pub.publish(current);
	}
	else if(followWall)
	{
		
	
	
	}
	else
	{
		std::cout << "STOPPING\n" << std::endl;
		current.linear.x = 0;
		current.angular.z = msg.angular.z;
		velocity_pub.publish(current);
		wallFollow = true;
	}
}

void scanCallback(const sensor_msgs::LaserScan& msg){

	//geometry_msgs::Twist stop;
	//stop.linear.x = 0;

	//shallow copy, only meant to be pedantic.
	std::vector<float> myvector = msg.ranges;


    double stop_Min = msg.range_min;
	double stop_Max = msg.range_max;
	double threshold = 0.16;
	
	
	std::cout << "scanCallback\n";

    std::cout << "myvector contains:";
	safeToMove = true;
   for (std::vector<float>::iterator it=myvector.begin(); it!=myvector.end(); ++it){

	//the current value we're iterating through.
        float val = *it;
	
 
	if(val >= stop_Min && val <= stop_Max )
    {
			 
		  if(val <= 0.4 && val >= 0.30){
		  		safeToMove = false;  
		  		break;	  
		  }
		  else
		  {
		  		safeToMove = true;
		  //    stop.linear.x = 0.25;
		  }	

		}

	}
	
//	velocity_pub.publish(stop);

}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "follower");
	ros::NodeHandle n;
	
	std::cout << "AVOIDANCE IS RUNNING" << std::endl;
	// advertise that we will publish cmd_vel messages
	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// subscribe to blob messages and call blobCallback when they are received
	//ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);   
	
	ros::Subscriber scanSub = n.subscribe("scan",1000,scanCallback);
	ros::Subscriber teleopSub = n.subscribe("cmd_vel_unsafe",1000,unsafeCallback);
	
	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}


