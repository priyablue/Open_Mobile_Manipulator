#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"

int main(int argc, char **argv)
{
    

	ros::init(argc, argv, "arrayPublisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int16MultiArray>("joint_states", 100);

	while (ros::ok())
	{
		std_msgs::Int16MultiArray array;
		//Clear array
		array.data.clear();
		//for loop, pushing data in the size of the array
                array.data.push_back(135); //195
                array.data.push_back(190);
                array.data.push_back(225);
                array.data.push_back(225);
                array.data.push_back(190);
                array.data.push_back(210);
		//Publish array
		pub.publish(array);
		//Let the world know
		ROS_INFO("I published something!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

}
