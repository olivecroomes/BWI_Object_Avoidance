#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
bool safeToMove = true;
ros::Publisher velocity_pub;
double speedFactor = 1.0;
//the robot will keep running as long as there are

//Point of interest - check type of parameter
//Hope this works!
void unsafeCallback(const geometry_msgs::Twist& msg) {
	

	geometry_msgs::Twist current;
	if (safeToMove) {
		//Can you believe this was the problem the whole time?
		current.linear.x = speedFactor * msg.linear.x;
		current.linear.y = msg.linear.y;
		current.linear.z = msg.linear.z;
		current.angular = msg.angular;
		velocity_pub.publish(current);
	} else {
		
		current.linear.x = 0;
		current.linear.y = msg.linear.y;
		current.linear.z = msg.linear.z;
		current.angular = msg.angular;
		velocity_pub.publish(current);
	}
}
/*
void depthCallback(){

}
*/
void scanCallback(const sensor_msgs::LaserScan& msg) {

	
	safeToMove = true;
	std::vector<float> myvector = msg.ranges;
	
	for (std::vector<float>::iterator it = myvector.begin();
			it != myvector.end(); ++it) {

		//the current value we're iterating through.
		float val = *it;

		//eliminates 0's
		if (val >= msg.range_min && val <= msg.range_max) {

			if (0.2 <= val && val <= 0.65) {
				safeToMove = false;
				break;
			}

		}
	}
								
	double sum = std::accumulate(myvector.begin(), myvector.end(), 0.0);
	double mean = sum / myvector.size();
	speedFactor = (1 - (mean / msg.range_max));
	

}

int main(int argc, char **argv) {
	ros::init(argc,argv,"Avoidance_Node");
	ros::NodeHandle n;
	
	std::cout << "AVOIDANCE IS RUNNING" << std::endl;
	ros::Subscriber scanSub = n.subscribe("scan", 1000,
			scanCallback);
	ros::Subscriber teleopSub = n.subscribe("cmd_vel_unsafe", 1000,
			unsafeCallback);
			
	//ros::Subscriber kinectSub = n.subscribe("",1000,depthCallback);
	
	velocity_pub = n.advertise < geometry_msgs::Twist > ("cmd_vel", 1000);

	ros::Rate loop_rate(100);

	ros::spin();

	return 0;
}

