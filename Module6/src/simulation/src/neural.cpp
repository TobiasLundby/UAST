#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

using namespace std;

// state subscriber
mavros_msgs::State state;
void stateSub(const mavros_msgs::State::ConstPtr &msg){
	state = *msg;
}

// Pose subscriber
geometry_msgs::PoseStamped localPose;
void localPoseSub(const geometry_msgs::PoseStamped::ConstPtr &msg){
	localPose = *msg;
}

// Enable offboard mode
bool enableOffboard(ros::ServiceClient &setModeClient, ros::Time &last){
	mavros_msgs::SetMode offBoard;
	offBoard.request.custom_mode = "OFFBOARD";
	if (setModeClient.call(offBoard) && offBoard.response.success){
		ROS_INFO("Offboard enabled");
		last = ros::Time::now();
		return true;
	}
	last = ros::Time::now();
	return false;
}

// Arm the drone
bool arm(ros::ServiceClient &armingClient, ros::Time &last){
	mavros_msgs::CommandBool arm;
	arm.request.value = true;
	if (armingClient.call(arm) && arm.response.success){
		ROS_INFO("Vehicle armed");
		last = ros::Time::now();
		return true;
	}
	last = ros::Time::now();
	return false;
}

int main(int argc, char **argv){

	// Init ros node
	ros::init(argc, argv, "simulation_node");
	ros::NodeHandle nh;
	ros::Rate rate(200);

	// Setup ros subscribtions
	ros::Subscriber mavrosStateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateSub);
	ros::Subscriber localposeSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPoseSub);

	// Setup ros publisher
	ros::Publisher posPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher angVelPub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 10);
	ros::Publisher throttle = nh.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 10);

	// Setup ros service clients
	ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	//ros::ServiceClient resetWorld = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

	// Save time stamp
	ros::Time last = ros::Time::now();

	while (ros::ok()){

		// Keep armed and offboard
		if (state.mode != "OFFBOARD" && (ros::Time::now() - last > ros::Duration(5.0))) enableOffboard(setModeClient, last);
		else if (!state.armed && (ros::Time::now() - last > ros::Duration(5.0))) arm(armingClient, last);

		// Set throttle
		std_msgs::Float64 thr;
		thr.data = 0.7;
		throttle.publish(thr);

		// Spin ros
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
