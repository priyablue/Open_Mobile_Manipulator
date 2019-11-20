#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int16MultiArray.h"


class Driver_Pub_Sub
{
public:
    Driver_Pub_Sub()
    {
        ros::Publisher pub_ = nh_.advertise<std_msgs::Int16MultiArray>("servo_cmd", 10);
	ros::Subscriber sub_= nh_.subscribe("/joint_states", 10, &Driver_Pub_Sub::JointState_Callback, this);
    }

    int transform(float x, float y){
        return int((x+1.57)*450.0/3.14 + y);
    }

    void JointState_Callback(const sensor_msgs::JointState::ConstPtr& joint_state){
        std_msgs::Int16MultiArray array;
    	array.data.clear();
    	array.data.push_back(transform(joint_state->position[0],-30));
    	array.data.push_back(transform(joint_state->position[1],-65));
    	array.data.push_back(450 - transform(joint_state->position[0],0));
    	array.data.push_back(450 - transform(joint_state->position[0],0));
    	array.data.push_back(transform(joint_state->position[0],-35));
    	array.data.push_back(450 - transform(joint_state->position[0],-15));
        ROS_INFO("INSIDE CALLBACK");
        pub_.publish(array);
	}
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};




int main(int argc, char** argv){
    ros::init(argc, argv, "driver_to_arduino");
    Driver_Pub_Sub Driver_obj;
    ros::spin();
    return 0;
}
