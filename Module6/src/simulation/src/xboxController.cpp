#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include "../include/ImgConverter.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sigc++/signal.h>
#include <sigc++/connection.h>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <string>
#include <mutex>
#include <linux/fs.h>

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
	ros::Rate rate(200);

	// Setup ros subscribtions
	ros::Subscriber mavrosStateSub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateSub);
	ros::Subscriber localposeSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, localPoseSub);

	// Setup ros publisher
	ros::Publisher localPosPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Publisher angVelPub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 10);
	ros::Publisher throttle = nh.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 10);

	// Setup ros service clients
	ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::ServiceClient resetWorld = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

	ImageConverter imageConv("/iris/camera/image_raw", 0);

	// Setup starting pose for the drone
	startDrone(localPosPub);

	// Save time stamp
	ros::Time last = ros::Time::now();

	int file;
	char name[128];

	int leftX = 0;
	int leftY = 0;
	int rightX = 0;
	int rightY = 0;

	if ((file = open("/dev/input/js1", O_RDONLY)) < 0){
		printf("Missing device!\n");
		return 1;
	}

	double X_pre_error = 0;
	double Y_pre_error = 0;

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

		struct js_event event;
		read(file, &event, sizeof(event));

		if (event.type & JS_EVENT_AXIS){
			if (event.number == 0) leftX = event.value;
			if (event.number == 1) leftY = event.value;
			if (event.number == 3) rightX = event.value;
			if (event.number == 4) rightY = event.value;
			std::cout << event.value << std::endl;
			if (event.number == 7 && event.value == pow(2, 15) - 1){
				std_srvs::Empty empty;
				resetWorld.call(empty);
			}
		}

		geometry_msgs::TwistStamped twist;
		twist.twist.angular.x = (double)rightX / pow(2, 15) * 2;
		twist.twist.angular.y = -(double)rightY / pow(2, 15) * 2;
		twist.twist.angular.z = -(double)leftX / pow(2, 15) * 2;

		/////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

    float K_p = 0.001;

    float K_i = 0.0;
    float dt = 0.1;
    float X_integral = 0.0;
    float Y_integral = 0.0;

    float K_d = 0.000;
    float X_derivative = 0.0;
    float Y_derivative = 0.0;

    float X_p = 0.0;
    float Y_p = 0.0;
    float X_i = 0.0;
    float Y_i = 0.0;
    float X_d = 0.0;
    float Y_d = 0.0;
		float X, Y;

    //Proportional term, error * K_p
    X_p = (twist.twist.angular.x)*K_p;
    Y_p = (twist.twist.angular.y)*K_p;
    // Integrational term
    X_integral += (twist.twist.angular.x) * dt;
    X_i = K_i * X_integral;

    Y_integral += (twist.twist.angular.y) * dt;
    Y_i = K_i * Y_integral;

    // Derivative term
    X_derivative = (twist.twist.angular.x - X_pre_error) / dt;
    X_d = K_d * X_derivative;

    Y_derivative = (twist.twist.angular.y - Y_pre_error) / dt;
    Y_d = K_d * Y_derivative;

    //Total output
    X -= X_p + X_i + X_d;
    Y -= Y_p + Y_i + Y_d;

    X_pre_error = twist.twist.angular.x;
    Y_pre_error = twist.twist.angular.y;

    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

		geometry_msgs::TwistStamped twist1;
		twist1.twist.angular.x = X;
		twist1.twist.angular.y = Y;
		twist1.twist.angular.z = -(double)leftX / pow(2, 15);
		angVelPub.publish(twist);

		std_msgs::Float64 thr;
		thr.data = 1 - ((double)leftY + pow(2, 15)) / pow(2, 16);
		throttle.publish(thr);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
