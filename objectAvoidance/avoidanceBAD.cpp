#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cmvision/Blobs.h"

ros::Publisher velocity_pub;

void scanCallback(const cmvision::Hokuyo& msg){
  geometry_msgs::Twist output;

  std::cout<<"recieved message: "<< msg->header << std::endl;

        output.linear.x = 0;
        output.angluar.z = 0;
	velocity_pub.publish(output);
}


}


int main(int argc, char **argv){

	ros::init(argc, argv, "follower");
	ros::NodeHandle n;
	
	std::cout<<"starting node"<< std::endl;
	
	// advertise that we will publish cmd_vel messages
	velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	// subscribe to blob messages and call scanCallback when they are received   
	
	ros::Subscriber sub = n.subscribe("scan",1000,scanCallback);
	
	ros::Rate loop_rate(10);

	ros::spin();


	return 0;
}
