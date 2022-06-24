#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "danceInstructor");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle4/waypoint", 1);

  ros::Rate loop_rate(10);

	int count = 0;
  while (ros::ok())
  {
	geometry_msgs::Twist twist;
	
	if(count < 20){
		twist.linear.x = 0;
		twist.linear.y = 0;
	}else if(count < 40){
		twist.linear.x = -0.3;
		twist.linear.y = 0;
	}else if(count < 60){
		twist.linear.x = -0.3;
		twist.linear.y = 0.3;
	}else{
		twist.linear.x = 0;
		twist.linear.y = 0.3;
	}

	ROS_INFO("Count: %d", count);

	chatter_pub.publish(twist);

	ros::spinOnce();

	loop_rate.sleep();
	count++;
	if(count > 80){
		count = 0;
	}
  }

  return 0;
}

