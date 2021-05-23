#include "ros/ros.h"
#include "handControl/hand_control_cmd.h"
#include "SerialClass.hpp"
#include <cstdlib>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Rate rate(200);

 	ros::Publisher command_pub = n.advertise<handControl::hand_control_cmd>("handControlCmd",1000);
	ros::Rate loop_rate(1);
	int count = 1;

	while(ros::ok())
	{

		handControl::hand_control_cmd msg;
		
		std::string str;
		str = argv[1];
		
		msg.msgtype = str.c_str();
		msg.value = atoll(argv[2]);

		ROS_INFO("Talker: msgtype = %s,value = %d",msg.msgtype.c_str(),msg.value);
		//ROS_INFO("Talker: msgtype = %s",msg.msgtype.c_str());
		command_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

	}

  	return 0;
}
