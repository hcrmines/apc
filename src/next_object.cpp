#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "apc/AddScoop.h"
#include <iostream>
#include <string>


geometry_msgs::Quaternion getQuaternion(geometry_msgs::Vector3 normal){
	geometry_msgs::Quaternion quatPose;
	/* Quaternion equation:
	   quat.w = cos(angle/2.0)
	   quat.x = vectorX*sin(angle/2.0)
	   quat.y = vectorY*sin(angle/2.0)
	   quat.z = vectorZ*sin(angle/2.0)
	 */
	// rotation of from a unit vector along z to the normal
	if(normal.z == -1 && normal.x == 0 && normal.y ==0){
		quatPose.x = 0.0;
		quatPose.y = 1.0;
		quatPose.z = 0.0;
		quatPose.w = 0.0;
	}
	else{
		double magnitude = sqrt(pow(normal.x,2)+pow(normal.y,2)+pow(normal.z,2));
		normal.x /= magnitude;
		normal.y /= magnitude;
		normal.z /= magnitude;
		quatPose.x = -normal.y;
		quatPose.y = normal.x;
		quatPose.z = 0.0;
		quatPose.w = 1.0 + normal.z;
		double newMag = sqrt(pow(quatPose.x,2)+pow(quatPose.y,2)+pow(quatPose.z,2)+pow(quatPose.w,2));
		quatPose.x /= newMag;
		quatPose.y /= newMag;
		quatPose.z /= newMag;
		quatPose.w /= newMag;
	}
	return quatPose;
}

int main(int argc, char **argv){
	ROS_INFO("Initializing next_object node");
	ros::init(argc, argv, "next_object_node");
	ros::NodeHandle nh;

	ros::Publisher pickPublisher = nh.advertise<geometry_msgs::Pose>("move_point", 1);
	ros::Publisher scoopPublisher = nh.advertise<apc::AddScoop>("add_scoop", 1);

	std_msgs::String msg;
	geometry_msgs::Vector3 testNorm;
	testNorm.x = 0.0;
	testNorm.y = 0.0;
	testNorm.z = -1.0;
	geometry_msgs::Pose testPose;
//	testPose.position.x = 0.7;
	testPose.position.x = 0.80;
	testPose.position.y = -0.4;
	testPose.position.z = 0.35;
		testPose.orientation.w = 0.7071;
		testPose.orientation.x = 0.0;
		testPose.orientation.y = 0.7071;
		testPose.orientation.z = 0.0;
//	testPose.orientation = getQuaternion(testNorm);
	ROS_INFO("quaternion is: %f %f %f %f", testPose.orientation.x, testPose.orientation.y, testPose.orientation.z, testPose.orientation.w);
	std::string input;
	apc::AddScoop scoopMessage;
	scoopMessage.add.data = true;
	while(nh.ok() && std::cin){
		std::cout << "Enter the next object: ";
		std::getline(std::cin, input);
		//msg.data = input;
		//pickPublisher.publish(msg);
		//pickPublisher.publish(testPose);
		scoopMessage.arm.data = input;
		scoopPublisher.publish(scoopMessage);

		ros::spinOnce();
	}
	return 0;
}
