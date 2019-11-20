#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>

void moveit_goal_Callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& traj){
	ROS_INFO("%d", int(traj->goal.trajectory.points.size()));//[0].positions[0]));
    for (int i=0; i < traj->goal.trajectory.points.size(); i++){
    	ROS_INFO("%f", traj->goal.trajectory.points[i].positions[0]);
        ROS_INFO("%f", traj->goal.trajectory.points[i].positions[1]);
        ROS_INFO("%f", traj->goal.trajectory.points[i].positions[2]);
        ROS_INFO("%f", traj->goal.trajectory.points[i].positions[3]);
        ROS_INFO("%f", traj->goal.trajectory.points[i].positions[4]);
        ROS_INFO("%f", traj->goal.trajectory.points[i].positions[5]);
        ROS_INFO("NEW NEW NEW");

    }
}


int main(int argc, char** argv){
	ros::init(argc, argv, "driver_to_arduino");
	ros::NodeHandle nh;
	ros::Subscriber sub= nh.subscribe("/arm_controller/follow_joint_trajectory/goal", 10, moveit_goal_Callback);
	ros::spin();
	return 0;
}