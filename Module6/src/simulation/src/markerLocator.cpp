#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include "../include/ImgConverter.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

// Setup starting pose for the drone
void startDrone(ros::Publisher &localPosPub){
	geometry_msgs::PoseStamped startPose;
	startPose.pose.position.x = 0;
	startPose.pose.position.y = 0;
	startPose.pose.position.z = 2;

	for (int i = 0; i < 100; i++){
		localPosPub.publish(startPose);
		ros::spinOnce();
		ros::Rate rate(20);
		rate.sleep();
	}
}

bool findBall(cv::Point2f &point, ImageConverter &imageConv){
	cv::Mat img = imageConv.get_image();

	cv::Mat binary;
	cv::inRange(img, cv::Scalar(0, 0, 200), cv::Scalar(10, 10, 255), binary);

	std::vector<std::vector<cv::Point> > cont;
	std::vector<cv::Vec4i> hier;
	cv::findContours(binary, cont, hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	if (!cont.size()) return false;

	cv::Moments moments = cv::moments(cont[0], false);
	point = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);

	cv::circle(img, point, 10, cv::Scalar(0, 255, 0), 2);
	cv::circle(img, cv::Point2f(img.cols / 2, img.rows / 2), 25, cv::Scalar(0, 0, 255), 2);

	point.x -= img.cols / 2;
	point.y -= img.rows / 2;

	cv::imshow("Image", img);
	cv::waitKey(1);
	return true;
}

void updatePose(ros::Publisher &localPosPub, ImageConverter &imageConv){
	static double pk = 0.002;

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;

	cv::Point2f center(0, 0);
	if (findBall(center, imageConv)){
		pose.pose.position.x = localPose.pose.position.x - center.y * pk;
		pose.pose.position.y = localPose.pose.position.y - center.x * pk;
		pose.pose.position.z = 2;
	}
	localPosPub.publish(pose);
}

int main(int argc, char **argv){

	// Init ros node
	ros::init(argc, argv, "simulation_node");
	ros::NodeHandle nh;
	ros::Rate rate(20);

	// Setup ros subscribtions
	ros::Subscriber mavrosStateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateSub);
	ros::Subscriber localposeSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPoseSub);

	// Setup ros publisher
	ros::Publisher localPosPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	// Setup ros service clients
	ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ImageConverter imageConv("/iris/camera/image_raw", 0);

	// Setup starting pose for the drone
	startDrone(localPosPub);

	// Save time stamp
	ros::Time last = ros::Time::now();

	while (ros::ok()){

		// Keep armed and offboard
		if (state.mode != "OFFBOARD" && (ros::Time::now() - last > ros::Duration(5.0))){
			mavros_msgs::SetMode offBoard;
			offBoard.request.custom_mode = "OFFBOARD";
      if (setModeClient.call(offBoard) && offBoard.response.success){
        ROS_INFO("Offboard enabled");
      }
      last = ros::Time::now();
    }
		else {
      if (!state.armed && (ros::Time::now() - last > ros::Duration(5.0))){
				mavros_msgs::CommandBool arm;
				arm.request.value = true;
				if (armingClient.call(arm) && arm.response.success) ROS_INFO("Vehicle armed");
				last = ros::Time::now();
      }
    }

		updatePose(localPosPub, imageConv);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
