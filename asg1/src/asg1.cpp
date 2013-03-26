/*
By Josh Eversmann, jme2347 
   and Robert Lynch, rml953
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"

ros::Publisher velocity_pub;

// This method is called whenever a blob message is received
void blobCallback(const cmvision::Blobs::ConstPtr& msg){

	// This is the output velocity that we will publish
	geometry_msgs::Twist output;

	// check if any blobs were found
	if (msg->blob_count > 0){

		double largeArea = 400, centerX = 320;

		for (int i = 0; i < msg->blob_count; i++){

			// Print blob info
			std::cout << "Detected blob " << i << " with area " << msg->blobs[i].area << std::endl;

			// Get area and center for largest blob			
			if(msg->blobs[i].area > largeArea) {
				largeArea = msg->blobs[i].area;      // blob area
				centerX = msg->blobs[i].x;         // blob center x
			}

			/*		
			msg->blobs[i].y;         // blob center y
			msg->blobs[i].left;      // blob left x
			msg->blobs[i].right;     // blob right x
			msg->blobs[i].top;       // blob top x
			msg->blobs[i].bottom;    // blob bottom x
			*/
		}


		double x = (largeArea > 400) ? (10000 - largeArea)/10000 : 0; // foward velocity
		
		output.linear.x = x;
		output.angular.z = -(centerX - 320)/480; // angular velocity

		velocity_pub.publish(output); // publish this velocity message
	}
}


int main(int argc, char **argv){

	ros::init(argc, argv, "follower");
	ros::NodeHandle n;

	// advertise that we will publish cmd_vel messages
	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// subscribe to blob messages and call blobCallback when they are received
	ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);   

	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}


