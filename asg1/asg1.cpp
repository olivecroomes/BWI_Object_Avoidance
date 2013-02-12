#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"

ros::Publisher velocity_pub;

void blobCallback(const cmvision::Blobs::ConstPtr& msg){
	geometry_msgs::Twist output;


/*****************************
*     Insert code here
******************************/

        std::cout << "got blob message, count: " << msg->blob_count << std::endl;

	if (msg->blob_count > 0){
	
	output.linear.x = .3;
	output.angular.x = .3;



	velocity_pub.publish(output);
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "follower");
	ros::NodeHandle n;

	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("blobs", 1000, blobCallback);   

	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}


