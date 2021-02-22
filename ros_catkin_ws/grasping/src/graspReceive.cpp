#include "ros/ros.h"
#include "std_msgs/Int64.h"

void chatterCallback(const std_msgs::Int64::ConstPtr& msg)
{
  	ROS_INFO("I heard: [%d]", int(msg->data));
	system("canberra-gtk-play -f beep.wav");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graspReceive");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("eeg", 1000, chatterCallback);
  ros::spin();

  return 0;
}
